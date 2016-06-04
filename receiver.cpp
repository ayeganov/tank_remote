#include <boost/asio.hpp>
#include "robo_utils.hpp"
#include "azmq/socket.hpp"



void usage ( int argc, char *argv[] )
{
    printf("Usage:\n\t%s key\n\nvalid keys are:\n\tlshift\t- Left Shift key\n" , argv[0]);

    exit(EXIT_FAILURE);
}

void got_keys(std::string&& keys)
{
    std::cout << "Got keys: " << keys << std::endl;
}

int main (int argc, char *argv[], char *env[])
{
    boost::asio::io_service io;
    roboutils::AzmqSock<azmq::sub_socket, 256> key_sub{io};
    key_sub.get_socket().connect("tcp://127.0.0.1:9000");
    key_sub.get_socket().set_option(azmq::socket::subscribe());

    key_sub.on_recv(got_keys);

    io.run();
    return 0;
}
