#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include <linux/input.h>

void usage ( int argc, char *argv[] )
{
    printf("Usage:\n\t%s key\n\nvalid keys are:\n\tlshift\t- Left Shift key\n" , argv[0]);

    exit(EXIT_FAILURE);
}

int main (int argc, char *argv[], char *env[])
{
    if ( argc != 2 )    usage(argc, argv);

    int key;

    if ( strcmp(argv[1], "lshift") == 0 )       key = KEY_LEFTSHIFT;
    else if ( strcmp(argv[1], "rshift") == 0 )  key = KEY_RIGHTSHIFT;
    else if ( strcmp(argv[1], "lalt") == 0 )    key = KEY_LEFTALT;
    else if ( strcmp(argv[1], "ralt") == 0 )    key = KEY_RIGHTALT;
    else if ( strcmp(argv[1], "lctrl") == 0 )   key = KEY_LEFTCTRL;
    else if ( strcmp(argv[1], "rctrl") == 0 )   key = KEY_RIGHTCTRL;

    int kbd = open("/dev/input/by-path/platform-i8042-serio-0-event-kbd", O_RDONLY);

    uint8_t key_map[KEY_MAX/8 + 1];    //  Create a byte array the size of the number of keys
    memset(key_map, 0, sizeof(key_map));    //  Initate the array to zero's
    ioctl(kbd, EVIOCGKEY(sizeof(key_map)), key_map);    //  Fill the keymap with the current keyboard state

    int enter_byte = key_map[KEY_ENTER/8];  //  The key we want (and the seven others arround it)
    int enter_mask = 1 << (KEY_ENTER % 8);  //  Put a one in the same column as out key state will be in;
    int shift_byte = key_map[KEY_LEFTSHIFT / 8];
    int shift_mask = 1 << (KEY_LEFTSHIFT % 8);
    if(enter_byte & enter_mask)
    {
        printf("Enter is pressed.\n");
    }
    if(shift_byte & shift_mask)
    {
        printf("Left shift is pressed.\n");
    }

    return 0;
}
