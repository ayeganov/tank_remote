#ifndef _PERIODIC_CALLBACK_
#define _PERIODIC_CALLBACK_

#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <array>
#include <functional>
#include <future>


using namespace boost;

namespace roboutils
{

/**
 * Schedules the given callback to be called periodically.
 *
 * The callback is called every duration (posix_time::seconds, or others).
 *
 * If the callback runs for longer than duration, subsequent
 * invocations will be skipped to get back on schedule.
 *
 * Start must be called after the PeriodicCallback is created.
 */
class PeriodicCallback
{
public:
    PeriodicCallback(posix_time::time_duration duration, asio::io_service& io)
    : m_timer(io),
      m_duration(duration)
    {
    }

    template<class F, class... Args>
    void start(F&& f, Args&&... args)
    {
        m_callback = std::bind(f, std::forward<Args>(args)...);
        schedule_callback();
    }

private:

    void schedule_callback()
    {
        m_timer.expires_from_now(m_duration);
        m_timer.async_wait(bind(&PeriodicCallback::handle_timeout, this, asio::placeholders::error));
    }

    void handle_timeout(system::error_code const& error)
    {
        if(error.value() == asio::error::operation_aborted)
        {
            return;
        }

        m_callback();
        schedule_callback();
    }

    asio::deadline_timer m_timer;
    posix_time::time_duration m_duration;
    std::function<void()> m_callback;
};


template <typename SockType, size_t buf_size>
class AzmqSock
{
    public:
        AzmqSock(asio::io_service& loop)
        : m_socket(loop),
          m_buffer()
        {
        }

        void bind(std::string address)
        {
            m_socket.bind(address);
        }

        void connect(std::string address)
        {
            m_socket.connect(address);
        }

        void handle_receive(system::error_code ec, size_t bytes_transferred)
        {
            if(ec)
            {
                std::cerr << ec.message() << std::endl;
                return;
            }
            std::string tmp{m_buffer.data(), bytes_transferred};
            m_callback(std::move(tmp));
            m_socket.async_receive(asio::buffer(m_buffer), [this](system::error_code ec, size_t bt) {
                handle_receive(ec, bt);
            });
        }

        template<class F>
        void on_recv(F&& f)
        {
            m_callback = std::move(f);
            m_socket.async_receive(asio::buffer(m_buffer), [this](system::error_code ec, size_t bt) {
                handle_receive(ec, bt);
            });
        }

    private:
        SockType m_socket;
        std::array<char, buf_size> m_buffer;
        std::function<void(std::string)> m_callback;
};

};
#endif
