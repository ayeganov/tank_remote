#include <cstdint>

#include <array>
#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include <termios.h>
#include <unistd.h>
#include <linux/input.h>


class KeyboardState
{
    public:
        using KeyToCharMap = std::map<int, char>;
        using KeyBitArray = std::array<uint8_t, KEY_MAX / 8 + 1>;

        KeyboardState(std::string keyb_path)
        : m_keyb_fd(0),
          m_key_map{0}
        {
            m_keyb_fd = open(keyb_path.data(), O_RDONLY);
            if(m_keyb_fd == -1)
            {
                std::cerr << "ERROR: " << std::strerror(errno) << '\n';
            }
        }

        KeyboardState(KeyBitArray kba)
        : m_key_map(kba)
        {

        }

        void update_keyboard_state()
        {
            m_key_map.fill(0);
            int result = ioctl(m_keyb_fd, EVIOCGKEY(m_key_map.size()), m_key_map.data());
            if(result == -1)
            {
                std::cerr << "ERROR: failed to update keyboard state."
                          << "\n\t" << std::strerror(errno) << std::endl;
            }
        }

        bool is_key_set(int key)
        {
            uint8_t key_bitset = m_key_map[key / 8];
            uint8_t key_mask = 1 << (key % 8);
            return bool(key_bitset & key_mask);
        }

        std::string state_as_string(KeyToCharMap vals)
        {
            std::string state;
            for(auto entry : vals)
            {
                if(is_key_set(entry.first))
                {
                    state += entry.second;
                }
            }
            return std::move(state);
        }

        ~KeyboardState()
        {
            if(is_ready())
            {
                close(m_keyb_fd);
            }
        }

        bool is_ready() { return m_keyb_fd > 0; }

    private:
        int m_keyb_fd;
        KeyBitArray m_key_map;
};


class Input : public boost::enable_shared_from_this<Input>
{
public:
    typedef boost::shared_ptr<Input> Ptr;

    static void create(boost::asio::io_service& io_service)
    {
        Ptr input(new Input(io_service));
        input->read();
    }

private:

    explicit Input(
            boost::asio::io_service& io_service)
         : _input(io_service),
           m_key_state("/dev/input/by-path/platform-i8042-serio-0-event-kbd")

    {
        _input.assign( STDIN_FILENO );
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

    void read_handler(
            const boost::system::error_code& error,
            const size_t bytes_transferred)
    {
        if ( error ) {
            std::cerr << "read error: " << boost::system::system_error(error).what() << std::endl;
            return;
        }

        m_key_state.update_keyboard_state();
        if ( _command != '\n' ) {
            std::cout << "command: " << _command << std::endl;
        }

        std::cout << m_key_state.state_as_string({ {KEY_W, 'w'},
                                                  {KEY_A, 'a'},
                                                  {KEY_S, 's'},
                                                  {KEY_D, 'd'}
                                                }) << std::endl;
        if(m_key_state.is_key_set(KEY_W))
        {
            std::cout << "You pressed W\n";
        }
        this->read();
    }

private:
    boost::asio::posix::stream_descriptor _input;
    char _command;
    KeyboardState m_key_state;
};


class NonCanonicalStdin
{
    public:
        NonCanonicalStdin() : m_term(), m_old_term(), m_success(false) {}

        bool init()
        {
            std::cout << "Retrieving the terminal." << std::endl;
            std::cout << "stdin " << STDIN_FILENO << std::endl;
            if(tcgetattr(STDIN_FILENO, &m_term) && tcgetattr(STDIN_FILENO, &m_old_term))
            {
                std::cerr << "ERROR: tcgetttr failed\n";
                return -1;
            }
            // Switch from canonical mode to non-canonical: No line discipline, every
            // character typed triggers an event to whoever is polling.
            m_term.c_lflag &= ~ICANON;
            // Don't echo typed characters to the screen
            m_term.c_lflag &= ~ECHO;
            // At least one character has to be typed before poller gives up
            m_term.c_cc[VMIN] = 1;

            if(tcsetattr(STDIN_FILENO, TCSANOW, &m_term))
            {
                std::cerr << "ERROR: tcsetattr failed\n";
                return -1;
            }
            m_success = true;
            return m_success;
        }

        ~NonCanonicalStdin()
        {
            std::cout << "Restoring the terminal." << std::endl;
            if(m_success)
            {
                std::cout << "stdin " << STDIN_FILENO << std::endl;
                if(tcsetattr(fileno(stdin), TCSANOW, &m_old_term))
                {
                    std::cerr << "ERROR: failed to reset terminal settings\n";
                    std::cerr << std::strerror(errno) << std::endl;
                }
            }
        }

    private:
    struct termios m_term, m_old_term;
    bool m_success;
};


int main()
{
    {
        NonCanonicalStdin ncs;

        boost::asio::io_service io;
        boost::asio::signal_set signals(io, SIGINT, SIGTERM);
        signals.async_wait(boost::bind(&boost::asio::io_service::stop, &io));

        if(!ncs.init())
        {
            std::cerr << "Failed to set terminal settings.\n";
            exit(EXIT_FAILURE);
        }

        Input::create(io);

        io.run();
    }
    std::cout << "bye bye roboto" << std::endl;
    return 0;
}
