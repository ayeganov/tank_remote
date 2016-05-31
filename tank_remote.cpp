#include <cstdint>
#include <cstdio>
#include <iostream>
#include <map>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <termios.h>
#include <unistd.h>
#include <linux/input.h>


class Input : public boost::enable_shared_from_this<Input>
{
public:
    typedef boost::shared_ptr<Input> Ptr;

    static void create(boost::asio::io_service& io_service)
    {
        Ptr input(new Input(io_service));
        input->read();
    }

    ~Input()
    {
        if(_kbd_fd)
        {
            close(_kbd_fd);
        }
    }

private:

    explicit Input(
            boost::asio::io_service& io_service)
         : _input(io_service),
           _kbd_fd(0),
           _key_map{0}

    {
        _input.assign( STDIN_FILENO );
        _kbd_fd = open("/dev/input/by-path/platform-i8042-serio-0-event-kbd", O_RDONLY);
    }

    void read()
    {
        boost::asio::async_read(
                _input,
                boost::asio::buffer(&_command, sizeof(_command)),
                boost::bind(
                    &Input::read_handler,
                    shared_from_this(),
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred
                    )
                );
    }

    void update_kb_state()
    {
        std::cout << "Updating the key map\n";
        memset(_key_map, 0, sizeof(_key_map));
        ioctl(_kbd_fd, EVIOCGKEY(sizeof(_key_map)), _key_map);
    }

    void read_handler(
            const boost::system::error_code& error,
            const size_t bytes_transferred)
    {
        if ( error ) {
            std::cerr << "read error: " << boost::system::system_error(error).what() << std::endl;
            return;
        }

        update_kb_state();
        if ( _command != '\n' ) {
            std::cout << "command: " << _command << std::endl;
        }

        int w_byte = _key_map[KEY_W / 8];
        int w_mask = 1 << (KEY_W % 8);
        if(w_byte & w_mask)
        {
            std::cout << "You pressed W\n";
        }
        this->read();
    }

private:
    boost::asio::posix::stream_descriptor _input;
    char _command;
    int _kbd_fd;
    uint8_t _key_map[KEY_MAX / 8 + 1];
};


int make_stdin_non_canonical()
{
    struct termios term;
    if(tcgetattr(STDIN_FILENO, &term))
    {
        std::cerr << "ERROR: tcgetttr failed\n";
        return -1;
    }
    // Switch from canonical mode to non-canonical: No line discipline, every
    // character typed triggers an event to whoever is polling.
    term.c_lflag &= ~ICANON;
    // Don't echo typed characters to the screen
    term.c_lflag &= ~ECHO;
    // At least one character has to be typed before poller gives up
    term.c_cc[VMIN] = 1;

    if(tcsetattr(STDIN_FILENO, TCSANOW, &term))
    {
        std::cerr << "ERROR: tcsetattr failed\n";
        return -1;
    }
    return 0;
}


int main()
{
    boost::asio::io_service io;

    if(make_stdin_non_canonical())
    {
        exit(EXIT_FAILURE);
    }

    Input::create(io);

    io.run();

    return 0;
}
