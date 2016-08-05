#ifndef CPPTIME_H_
#define CPPTIME_H_

#include <cassert>
#include <chrono>
#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <array>
#include <stack>
#include <unistd.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>

namespace CppTime
{
// Public types
using timer_id = int;
using duration = std::chrono::microseconds;
using handler_t = std::function<void(timer_id)>;

namespace detail
{

const int max_events = 10;

struct Event {
	template <typename Func>
	Event(const itimerspec &tspec, Func &&handler)
	    : tspec(tspec), handler(std::forward<Func>(handler)), valid(true)
	{
	}
	Event(Event &&) = default;
	Event &operator=(Event &&) = default;
	Event(const Event &) = delete;
	Event &operator=(const Event &) = delete;
	struct itimerspec tspec;
	handler_t handler;
	bool valid;
};

} // end namespace detail

struct Null_mutex {
	void lock()
	{
	}
	void unlock() noexcept
	{
	}
	bool try_lock()
	{
		return true;
	}
};

class Timer
{

	// Thread and locking variables.
	using ulock = std::unique_lock<std::mutex>;
	std::mutex m;
	std::thread worker;

	// epoll
	int epollfd;
	int closefd;
	struct epoll_event ev;

	// A list of ids to be re-used. If possible, ids are used from this pool.
	std::stack<timer_id> free_ids;
	std::unordered_map<timer_id, detail::Event> events;
	std::array<struct epoll_event, detail::max_events> epoll_events;

	// Use to terminate the timer thread.
	std::atomic_bool done{false};

public:
	Timer() : m{}, worker{}, epollfd{-1}, free_ids{}, events{}, epoll_events()
	{
		// Initialize epoll
		epollfd = epoll_create(10); // 10 is arbitrary - must be larger than 0
		assert(epollfd != -1);

		// Initialize eventfd to finish epoll
		closefd = eventfd(0, 0);
		assert(closefd != -1);
		ev.events = EPOLLIN;
		ev.data.fd = closefd;
		int ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, closefd, &ev);
		assert(ret == 0);

		// Start worker thread that monitors timers
		worker = std::thread(std::bind(&Timer::run, this));
	}

	~Timer()
	{
		done = true;
		uint64_t buf = 1;
		write(closefd, &buf, sizeof(uint64_t));
		worker.join();
		for(auto &e : events) {
			close(e.first);
		}
		close(epollfd);
		events.clear();
		while(!free_ids.empty()) {
			free_ids.pop();
		}
	}

	inline timer_id add(
	    const duration &when, handler_t &&handler, const duration &period = duration::zero())
	{
		using namespace std::chrono;

		int fd = -1;

		struct timespec first {
			.tv_sec = duration_cast<seconds>(when).count(),
			.tv_nsec = duration_cast<nanoseconds>(when).count() % 1000000000
		};
		struct timespec dur {
			.tv_sec = 0, .tv_nsec = 0
		};
		if(period != duration::zero()) {
			dur.tv_sec = duration_cast<seconds>(period).count();
			dur.tv_nsec = duration_cast<nanoseconds>(period).count() % 1000000000;
		}
		struct itimerspec tspec {
			.it_interval = dur, .it_value = first
		};

		detail::Event e{tspec, handler};

		ulock l(m);
		if(free_ids.empty()) {
			fd = timerfd_create(CLOCK_MONOTONIC, 0);
			assert(fd != -1);
			events.insert(std::make_pair(fd, std::move(e)));
			ev.events = EPOLLIN;
			ev.data.fd = fd;
			int ret = epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &ev);
			assert(ret == 0);
			ret = timerfd_settime(fd, 0, &(events.at(fd).tspec), nullptr);
			assert(ret == 0);
		} else {
			fd = free_ids.top();
			free_ids.pop();
			events.erase(fd);
			events.insert(std::make_pair(fd, std::move(e)));
			ev.events = EPOLLIN;
			ev.data.fd = fd;
			int ret = epoll_ctl(epollfd, EPOLL_CTL_MOD, fd, &ev);
			assert(ret == 0);
			ret = timerfd_settime(fd, 0, &(events.at(fd).tspec), nullptr);
			assert(ret == 0);
		}

		return fd;
	}

	inline timer_id add(const uint64_t when, handler_t &&handler, const uint64_t period = 0)
	{
		return add(duration(when), std::move(handler), duration(period));
	}

	inline bool remove(timer_id id)
	{
		ulock l(m);
		const auto el = events.find(id);
		if(el == events.end()) {
			return false;
		}

		// Disarm the timer
		struct timespec none {
			0, 0
		};
		struct itimerspec tspec {
			none, none
		};
		events.at(id).tspec = tspec;
		events.at(id).valid = false;
		l.unlock();

		int ret = timerfd_settime(id, 0, &(events.at(id).tspec), NULL);
		assert(ret == 0);
		free_ids.push(id);
		return true;
	}

private:
	void run()
	{
		while(!done) {
			int nfds = epoll_wait(epollfd, &epoll_events[0], detail::max_events, -1);
			assert(nfds > 0);

			for(int i = 0; i < nfds; ++i) {
				if(epoll_events[i].events == EPOLLIN) {
					const auto fd = epoll_events[i].data.fd;
					uint64_t buf;
					read(fd, &buf, sizeof(uint64_t));

					if(fd == closefd && buf == 1) {
						// We received a close request. Terminate loop.
						break;
					}

					// This is a regular request. Call our registered handler.
					ulock l(m);
					const bool valid = events.at(fd).valid;
					const auto handler = events.at(fd).handler;
					l.unlock();
					if(!valid) {
						continue;
					}
					events.at(fd).handler(fd);
				}
			}
		}
	}
};

} // end namespace CppTime

#endif // CPPTIME_H_
