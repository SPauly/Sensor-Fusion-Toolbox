#ifndef SENSFUS_INTERNAL_EVENTBUS_H
#define SENSFUS_INTERNAL_EVENTBUS_H

#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>
#include <typeindex>

namespace sensfus {
namespace internal {

// Forward declaration
template <typename T>
class Channel;

/// @brief Wrapper for an existing Channel<T> to provide a publishing interface.
/// It should only be created via a call to EventBus::AddChannel().
/// @tparam T Event Type of the channel
template <typename T>
class Publisher {
 public:
  Publisher(std::shared_ptr<Channel<T>> channel) : channel_(channel) {}
  ~Publisher() = default;

  /// @brief Interface for publishing data to the channel.
  /// @param data data to be published to the channel. (This will create a
  /// shared_ptr to the data and publish it to all subscribers.)
  void Publish(const T& data) { channel_->Publish(data); }

 private:
  std::shared_ptr<Channel<T>> channel_;
};

/// @brief Channel<T> is a named channel for publishing and subscribing to
/// events of type T. It manages a queue of events and allows multiple
/// subscribers to receive the same events while ensuring that events read by
/// all subscribers are removed from the queue.
/// @tparam T Event Type of the channel
/// @details The Channel class is not copyable or movable. It should only be
/// created via a call to EventBus::AddChannel() or EventBus::Subscribe().
template <typename T>
class Channel {
 private:
  // Only allow publishing via the Publisher or EventBus classes
  friend class Publisher<T>;
  friend class EventBus;

 public:
  using DataPtr = std::shared_ptr<const T>;
  using Callback = std::function<void(const DataPtr&)>;

  Channel() = default;
  ~Channel() = default;

  // Channels are not copyable or movable
  Channel(const Channel&) = delete;
  Channel& operator=(const Channel&) = delete;
  Channel(Channel&&) = delete;
  Channel& operator=(Channel&&) = delete;

  /// @brief Manages a subscription to a Channel<T>. And provides a Fetch() data
  /// interface which will ensure proper deletion of the data, once it is not
  /// needed by any other subscriber.
  class Subscription {
   private:
    // Only the Channel can create a Subscription
    friend class Channel<T>;

   public:
    Subscription(Channel<T>* channel) : channel_(channel) {}
    ~Subscription() = default;

    // Subscription is not copyable or movable
    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;
    Subscription(Subscription&&) = delete;
    Subscription& operator=(Subscription&&) = delete;

    /// @brief Fetch the next data from the queue. If the queue is empty, it
    /// will return nullptr.
    /// @return Latest data or nullptr if the queue is empty.
    DataPtr Fetch() {
      std::lock_guard<std::mutex> lock(mtx_);
      if (queue_.empty()) return nullptr;
      auto data = queue_.front();
      queue_.pop();
      return data;
    }

   private:
    // Internal: called by Channel to deliver new data
    void deliver(const DataPtr& data) {
      std::lock_guard<std::mutex> lock(mtx_);
      queue_.push(data);
    }

   private:
    Channel<T>* channel_;
    std::queue<DataPtr> queue_;
    std::mutex mtx_;
  };

  /// @brief Subscribe to the channel. This will return a Subscription object
  /// which can be used to fetch data from the channel. This ensures that data
  /// that has been read by all Subscribers is removed from the queue.
  /// @return A Subscription object which can be used to fetch data from the
  /// channel.
  std::shared_ptr<Subscription> Subscribe() {
    auto sub = std::make_shared<Subscription>(this);
    std::lock_guard<std::mutex> lock(mtx_);
    subscribers_.push_back(sub);
    return sub;
  }

 private:
  /// @brief Method to Publish data to all subscribers. This is called by the
  /// Publisher class.
  /// @param data The data to be published to all subscribers.
  void Publish(const T& data) {
    auto ptr = std::make_shared<const T>(data);
    std::lock_guard<std::mutex> lock(mtx_);
    for (auto& sub : subscribers_) {
      if (sub) sub->deliver(ptr);
    }
  }

 private:
  std::vector<std::shared_ptr<Subscription>> subscribers_;
  std::mutex mtx_;
};

// EventBus: manages named channels of any type
class EventBus {
 public:
  EventBus() = default;
  ~EventBus() = default;

  /// @brief Main method to add a channel to the EventBus. This will create a
  /// new Channel<T> and return a Publisher<T> to it. If the channel already
  /// exists, it will return a Publisher<T> to the existing channel.
  /// @tparam T Event Type of the channel
  /// @param name Name of the channel to be created.
  /// @return A Publisher<T> object which can be used to publish data to the
  /// channel.
  template <typename T>
  std::shared_ptr<Publisher<T>> AddChannel(const std::string& name) {
    std::lock_guard<std::mutex> lock(mtx_);
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    if (channels_.count(key) == 0) {
      channels_[key] = std::make_shared<Channel<T>>();
    }

    return std::make_shared<Publisher<T>>(
        std::static_pointer_cast<Channel<T>>(channels_[key]));
  }

  /// @brief Publish data to a channel. It is faster to use a Publisher
  /// object to publish data, but this method is provided for convenience.
  /// @tparam T Event Type of the channel
  /// @param name Name of the channel to publish to.
  /// @param data Data to be published to the channel.
  /// @details This will do nothing if the channel does not exist.
  template <typename T>
  void Publish(const std::string& name, const T& data) {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = channels_.find(key);
    if (it != channels_.end()) {
      std::static_pointer_cast<Channel<T>>(it->second)->Publish(data);
    }
  }

  /// @brief Subscribe to a channel. This will return a Subscription object
  /// which provides a Fetch() data interface.
  /// @tparam T Event Type of the channel
  /// @param name Name of the channel to subscribe to.
  template <typename T>
  std::shared_ptr<typename Channel<T>::Subscription> Subscribe(
      const std::string& name) {
    auto key = std::make_pair(name, std::type_index(typeid(T)));
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = channels_.find(key);

    // Create the channel if it doesn't exist
    if (it == channels_.end()) {
      channels_[key] = std::make_shared<Channel<T>>();
      it = channels_.find(key);
    }

    // Cast the channel to the correct type and return the subscription
    return std::static_pointer_cast<Channel<T>>(it->second)->Subscribe();
  }

 private:
  using ChannelKey = std::pair<std::string, std::type_index>;
  struct ChannelKeyHash {
    std::size_t operator()(const ChannelKey& k) const {
      return std::hash<std::string>()(k.first) ^ k.second.hash_code();
    }
  };
  std::unordered_map<ChannelKey, std::shared_ptr<void>, ChannelKeyHash>
      channels_;
  std::mutex mtx_;
};

}  // namespace internal
}  // namespace sensfus

#endif  // SENSFUS_INTERNAL_EVENTBUS_H