#ifndef EVENT_H
#define EVENT_H

#include <deque>
#include <functional>
#include <memory>

namespace sensfus {
namespace app {
// forward declarations
class EventBase;
template <typename T>
class Event;

using EventCallbackType = std::function<void(std::unique_ptr<EventBase>)>;

enum EventType {};

class EventBase {
 public:
  EventBase() = default;
  EventBase(EventType etype) : event_type_(etype) {}
  virtual ~EventBase() {}

  inline const EventType &get_event_type_() { return event_type_; }

  template <typename T>
  T *GetData() {
    return static_cast<T *>(GetDataImpl());
  }

 protected:
  virtual void *GetDataImpl() { return nullptr; }

 private:
  EventType event_type_;
};

template <typename T>
class Event : public EventBase {
 public:
  Event(EventType etype, T *data, std::function<void(T)> *callback = nullptr)
      : EventBase(etype), data_(data), callback_(callback) {}
  ~Event() {}

 protected:
  virtual void *GetDataImpl() override { return data_; }

 private:
  T *data_;
  std::function<void(T)> *callback_;
};

inline void PushEvent(std::deque<std::unique_ptr<EventBase>> *que,
                      std::unique_ptr<EventBase> event) {
  que->push_back(std::move(event));
};

}  // namespace app
}  // namespace sensfus

#endif  // EVENT_H