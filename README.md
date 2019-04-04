C++ Timeouts for Posix
======================

A header-only C++11 timer component for Posix systems.

It manages a set of timeouts that, when expired, invoke a callback. It uses
features of C++11 and Posix.

It supports one-shot and periodic timeouts.

Documentation
-------------

Please see the documentation in [cpptime.h](./cpptime.h) for more detailed information about
the implementation. Also, this component provides the same interface as
[cpptime](https://github.com/eglimi/cpptime), but for Posix systems.

Implementation Status
---------------------

This is a new component and not much testing has been done. We already use it
in some of our products but expect that it receives some updates over time.

Examples
--------

A one shot timer.

~~~
using namespace std::chrono;
CppTime::Timer t;
t.add(seconds(2), [](CppTime::timer_id) { std::cout << "yes\n"; });
std::this_thread::sleep_for(seconds(3));
~~~

A periodic timer that is first executed after 2 seconds, and after this every
second. The event is removed after 10 seconds.

~~~
using namespace std::chrono;
CppTime::Timer t;
auto id = t.add(seconds(2), [](CppTime::timer_id) { std::cout << "yes\n"; }, seconds(1));
std::this_thread::sleep_for(seconds(10));
t.remove(id);
~~~

See the tests for more examples.

Usage
-----

To use the timer component, Simply copy [cpptime.h](./cpptime.h) into you
project. Everything is contained in this single header file.

Tests can be compiled and executed with the following commands, assuming you
are on a POSIX machine.

~~~
cd tests
g++ -g -std=c++11 -Wall -Wextra -o run_tests timer_test.cpp -l pthread
./tests
~~~

Contributions
-------------

Contributions, suggestions, and feature requests are welcome. Please use the
Github issue tracker.
