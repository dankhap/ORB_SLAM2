#ifndef QUEUE_H
#define QUEUE_H

#include <boost/thread/thread.hpp>
#include <queue>
#include <string>
template <typename Data> class Queue {
private:
  std::queue<Data> the_queue;
  mutable boost::mutex the_mutex;
  boost::condition_variable the_condition_variable;

public:
  void push(Data const &data) {
    boost::mutex::scoped_lock lock(the_mutex);
    the_queue.push(data);
    lock.unlock();
    the_condition_variable.notify_one();
  }

  bool empty() const {
    boost::mutex::scoped_lock lock(the_mutex);
    return the_queue.empty();
  }

  bool tryPop(Data &popped_value) {
    boost::mutex::scoped_lock lock(the_mutex);
    if (the_queue.empty()) {
      return false;
    }

    popped_value = the_queue.front();
    the_queue.pop();
    return true;
  }

  void waitAndPop(Data &popped_value) {
    boost::mutex::scoped_lock lock(the_mutex);
    while (the_queue.empty()) {
      the_condition_variable.wait(lock);
    }

    popped_value = the_queue.front();
    the_queue.pop();
  }
};

#endif /* QUEUE_H */
