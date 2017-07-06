#ifndef THSTACK_HPP
#define THSTACK_HPP

#include <exception>
#include <memory>
#include <mutex>
#include <stack>

using namespace std;

struct empty_stack : public std::exception {
  const char* what() const noexcept {
    return "empty stack exception happened";
  }
};

template<typename T>
class thstack {
  std::stack<T> data;
  mutable std::mutex m;

public:
  thstack() { };

  thstack(const thstack& other) {
    std::lock_guard<std::mutex> lock(other.m);
    data = other.data;
  }

  void push(T& v) {
    std::lock_guard<std::mutex> lock(m);
    data.push(v);
   // data.push(std::move(v));
   // std::cout << "push() & called " << std::endl;
  }

  void push(T&& v) {
    std::lock_guard<std::mutex> lock(m);
    // data.push(v);
    data.push(std::move(v));
   // std::cout << "push() && called " << std::endl;
  }

  std::shared_ptr<T> top() {
    std::lock_guard<std::mutex> lock(m);
    if(data.empty()) {
        std::cout << "###### " << std::endl;
      // throw empty_stack();
        return std::shared_ptr<T>();
    }
    std::shared_ptr<T> const res(std::make_shared<T>(data.top()));
    return res;
  }

  std::shared_ptr<T> pop() {

    std::lock_guard<std::mutex> lock(m);
    if(data.empty()) {
      // throw empty_stack();
        return std::shared_ptr<T>();
    }
    std::shared_ptr<T> const res(std::make_shared<T>(std::move(data.top())));
    data.pop();
    return res;
  }

  bool empty() const
  {
    std::lock_guard<std::mutex> lock(m);
    return data.empty();
  }

  size_t size() const
  {
    std::lock_guard<std::mutex> lock(m);
    return data.size();
  }

};


#endif // THSTACK_HPP
