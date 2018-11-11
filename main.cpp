#include <err.h>
#include <stdio.h>
#include <assert.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/uinput.h>
#include <string.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <pthread.h>
#include <atomic>

#define STR(X) STR2(X)
#define STR2(X) #X

#define ENSURE(CONDITION, ...) if (!(CONDITION)) { err(EXIT_FAILURE, __FILE__ ":" STR(__LINE__) " " __VA_ARGS__); }

static const char *EVENT_DEV_NAME = "event";
static const char *EVENT_DEV_PREFIX = "/dev/input/";
static const char *UINPUT_NAME = "/dev/uinput";

static const int BITS_PER_LONG = sizeof(long) * 8;

inline int nbytes(int x)
{
    return (x - 1) / BITS_PER_LONG + 1;
}

inline bool testBits(int bit, const unsigned long array[])
{
    unsigned long result = array[bit / BITS_PER_LONG] >> bit % BITS_PER_LONG & 1;
    return (result == 1);
}

static int isEventDevice(const struct dirent *dir) 
{
	return strncmp(EVENT_DEV_NAME, dir->d_name, strlen(EVENT_DEV_NAME)) == 0;
}

static void createVirtualMouse(int fd)
{
    // enable mouse button left and relative events
    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_KEYBIT, BTN_LEFT);

    ioctl(fd, UI_SET_EVBIT, EV_REL);
    ioctl(fd, UI_SET_RELBIT, REL_X);
    ioctl(fd, UI_SET_RELBIT, REL_Y);

    struct uinput_user_dev uud;
    memset(&uud, 0, sizeof(uud));

    snprintf(uud.name, UINPUT_MAX_NAME_SIZE, "Virtual Mouse");

    int ret = write(fd, &uud, sizeof(uud));
    ENSURE(ret == sizeof(uud));

    ioctl(fd, UI_DEV_CREATE);
}

static std::vector<std::string> getAllEventDevicePath()
{
    struct dirent **direntList;

    int ndev = scandir(EVENT_DEV_PREFIX, &direntList, isEventDevice, versionsort);
    ENSURE(ndev > 0, "No event device found");

    std::vector<std::string> result(ndev);

    for (int i = 0; i < ndev; i++)
    {
        result[i] = std::string(EVENT_DEV_PREFIX) + std::string(direntList[i]->d_name);
    }
    return std::move(result);
}

// A simple RAII class
class FileOpener
{
public:
    explicit FileOpener(const char *pathname, int flags)
    {
        fd = open(pathname, flags);
    }

    explicit FileOpener(const char *pathname, int flags, mode_t mode)
    {
        fd = open(pathname, flags, mode);
    }

    ~FileOpener()
    {
        if (fd >= 0)
        {
            close(fd);
        }
    }

    int getFd()
    {
        return fd;
    }

private:
    int fd;
};

static bool isSupportType(const std::string &path, int type)
{
    FileOpener opener(path.c_str(), O_RDONLY);
    int fd = opener.getFd();
    if (fd < 0)
    {
        return false;
    }

    unsigned long supportType[nbytes(EV_MAX)];
    ioctl(fd, EVIOCGBIT(0, EV_MAX), supportType);
    if (!testBits(type, supportType))
    {
        return false;
    }

    return true;
}

static bool isSupportTypeCode(const std::string &path, int type, int code)
{
    if (!isSupportType(path, type))
    {
        return false;
    }

    FileOpener opener(path.c_str(), O_RDONLY);
    int fd = opener.getFd();
    if (fd < 0)
    {
        return false;
    }

    unsigned long supportCode[nbytes(KEY_MAX)];
    ioctl(fd, EVIOCGBIT(type, KEY_MAX), supportCode);
    if (!testBits(code, supportCode))
    {
        return false;
    }
    return true;
}

static const int NEEDED_KEY[] = { KEY_KP8, KEY_KP2, KEY_KP4, KEY_KP6 };
// device reports KEY_KP8 KEY_KP2 KEY_KP4 KEY_KP6 is valid keyboard
static bool isValidKeyboard(const std::string &path)
{
    for (unsigned int i = 0; i < (sizeof(NEEDED_KEY) / sizeof(int)); i++)
    {
        if (!isSupportTypeCode(path, EV_KEY, NEEDED_KEY[i]))
        {
            return false;
        }
    }
    return true;
}

static std::vector<std::string> getAllValidKeyboard()
{
    std::vector<std::string> validDevice;
    std::vector<std::string> allDevice = getAllEventDevicePath();

    for (unsigned int i = 0; i < allDevice.size(); i++)
    {
        if (isValidKeyboard(allDevice[i]))
        {
            validDevice.push_back(allDevice[i]);
        }
    }

    return validDevice;
}

static std::string getOneNumLockDevice()
{
    std::vector<std::string> allDevice = getAllEventDevicePath();

    for (unsigned int i = 0; i < allDevice.size(); i++)
    {
        if (isSupportTypeCode(allDevice[i], EV_LED, LED_CAPSL))
        {
            return allDevice[i];
        }
    }
    return std::string("");
}

static int uinputFd = 0;

// program should exit
static volatile bool shouldStop = false;

// if numlock is on, we don't move mouse
static bool numlockOn = false;

const static int UP = 0;
const static int DOWN = 1;
const static int LEFT = 2;
const static int RIGHT = 3;
const static int DIRECTION_NUM = 4;

// move status
// change to true/false when corresponding key is pressed/released
static bool isMoving[DIRECTION_NUM] = { false };

// move times from the point that corresponding key is pressed
// reset to 0 when key is released
static std::atomic_int moveTimes[DIRECTION_NUM];

inline void resetMoveTimes()
{
    for (int i = 0; i < DIRECTION_NUM; i++)
    {
        moveTimes[i] = 0;
    }
}

// move step is affected by move times
// so we can apply some acceleration strategy
static int moveStep(int times)
{
    const int MIN_STEP = 1;
    const int MAX_STEP = 10;
    const int MIN_POINT = 50;
    const int MAX_POINT = 200;

    if (times <= MIN_POINT)
    {
        return MIN_STEP;
    }
    else if (times <= MAX_POINT)
    {
        return MIN_STEP + (times - MIN_POINT) * 1.0 / (MAX_POINT - MIN_POINT) * (MAX_STEP - MIN_STEP);
    }
    else
    {
        return MAX_STEP;
    }
}

// write mouse move event every timeInterval ms
static int timeInterval = 10;

static void setIsMoving(int index, int value)
{
    isMoving[index] = value;
    if (!value)
    {
        moveTimes[index] = 0;
    }
}

static void keyAction(int index, int value)
{
    ENSURE(index >= 0 && index < DIRECTION_NUM);
    if (value == 1)
    {
        setIsMoving(index, true);
    }
    else if (value == 0)
    {
        setIsMoving(index, false);
    }
    else
    {
        // do nothing
    }
}

static void handleKeyEvent(const struct input_event &ev)
{
    if (numlockOn)
    {
        return;
    }

    int index = -1;
    switch (ev.code)
    {
    case KEY_KP8: index = UP; break;
    case KEY_KP2: index = DOWN; break;
    case KEY_KP4: index = LEFT; break;
    case KEY_KP6: index = RIGHT; break;
    default: index = -1; break;
    }
    if (index != -1)
    {
        keyAction(index, ev.value);
    }
}

static void handleLEDEvent(const struct input_event &ev)
{
    if (ev.code == LED_NUML)
    {
        if (ev.value == 0)
        {
            numlockOn = false;
        }
        else if (ev.value == 1)
        {
            numlockOn = true;
            for (int i = 0; i < DIRECTION_NUM; i++)
            {
                setIsMoving(i, false);
            }
        }
    }
}

static void handleEvent(const struct input_event &ev)
{
    if (ev.type == EV_KEY)
    {
        handleKeyEvent(ev);
    }
    else if (ev.type == EV_LED)
    {
        handleLEDEvent(ev);
    }
}

static void handleDevice(int fd)
{
    struct input_event ev[64];
    int rd = read(fd, ev, sizeof(ev));
    ENSURE(rd % sizeof(struct input_event) == 0, "expected %d bytes, got %d\n", (int) sizeof(struct input_event), rd % (int) sizeof(struct input_event));
    int n = rd / sizeof(struct input_event);
    for (int j = 0; j < n; j++)
    {
        handleEvent(ev[j]);
    }
}

static void interrupt_handler(int sig)
{
    (void) sig;
	shouldStop = true;
}

static bool queryNumlock()
{
    std::string ledDevice = getOneNumLockDevice();

    // if there is no led device, we suppose that numlock is always off
    if (ledDevice == std::string(""))
    {
        return false;
    }

    FileOpener opener(ledDevice.c_str(), O_RDONLY);

    int fd = opener.getFd();
    ENSURE(fd >= 0);

    unsigned long supportLED[nbytes(LED_MAX)];
    ioctl(fd, EVIOCGLED(sizeof(supportLED)), supportLED);
    if (testBits(LED_NUML, supportLED))
    {
        return true;
    }
    else
    {
        return false;
    }
}

static void incMoveTimes()
{
    for (int i = 0; i < DIRECTION_NUM; i++)
    {
        if (isMoving[i])
        {
            moveTimes[i]++;
        }
    }
}

static void calcMoveSteps(int *steps)
{
    for (int i = 0; i < DIRECTION_NUM; i++)
    {
        if (isMoving[i])
        {
            steps[i] = moveStep(moveTimes[i]);
        }
    }

    // if opposite key is pressed, we set the steps all to 0
    if (isMoving[UP] && isMoving[DOWN])
    {
        steps[UP] = 0;
        steps[DOWN] = 0;
    }
    if (isMoving[LEFT] && isMoving[RIGHT])
    {
        steps[LEFT] = 0;
        steps[RIGHT] = 0;
    }
}

// ok, let's write device to move the mouse
static void writeDevice(int *steps)
{
    int x = steps[RIGHT] - steps[LEFT];
    int y = steps[DOWN] - steps[UP];

    struct input_event ev[3];
    memset(ev, 0, sizeof(ev));

    ev[0].type = EV_REL;
    ev[0].code = REL_X;
    ev[0].value = x;

    ev[1].type = EV_REL;
    ev[1].code = REL_Y;
    ev[1].value = y;

    ev[2].type = EV_SYN;
    ev[2].code = SYN_REPORT;

    int ret = write(uinputFd, ev, sizeof(ev));
    ENSURE(ret == sizeof(ev));
}

static void moveMouse()
{
    incMoveTimes();
    int steps[DIRECTION_NUM] = { 0 };
    calcMoveSteps(steps);
    writeDevice(steps);
}

static void *moveMouseThread(void *arg)
{
    (void) arg;

    while(true)
    {
        moveMouse();
        usleep(timeInterval * 1000);
    }

    return nullptr;
}

int main()
{
    uinputFd  = open(UINPUT_NAME, O_WRONLY);
    ENSURE(uinputFd >= 0);

    createVirtualMouse(uinputFd);

    numlockOn = queryNumlock();

    std::vector<std::string> kbds = getAllValidKeyboard();

    ENSURE(kbds.size() >= 1, "There should be at least one keyboard");

    struct pollfd fds[kbds.size()];
    memset(fds, 0, sizeof(fds));

    for (unsigned int i = 0; i < kbds.size(); i++)
    {
        fds[i].fd = open(kbds[i].c_str(), O_RDONLY);
        ENSURE(fds[i].fd >= 0, "Open keyboard %s failed", kbds[i].c_str());
        fds[i].events = POLLIN;
    }

    signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

    pthread_t tid;
    int ret = pthread_create(&tid, nullptr, moveMouseThread, nullptr);
    ENSURE(ret == 0);

    resetMoveTimes();

    while (!shouldStop)
    {
        for (unsigned int i = 0; i < kbds.size(); i++)
        {
            fds[i].revents = 0;
        }
        poll(fds, kbds.size(), -1);
        for (unsigned int i = 0; i < kbds.size(); i++)
        {
            if (fds[i].revents & POLLIN)
            {
                handleDevice(fds[i].fd);
            }
        }
    }

    for (unsigned int i = 0; i < kbds.size(); i++)
    {
        close(fds[i].fd);
    }
    return 0;
}