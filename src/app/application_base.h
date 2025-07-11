#ifndef APPLICATION_BASE_H
#define APPLICATION_BASE_H

namespace sensfus {
namespace app {

class ApplicationBase {
 public:
  explicit ApplicationBase() = default;
  virtual ~ApplicationBase() = default;

  virtual bool Run() = 0;

 protected:
  virtual bool Init() = 0;
  virtual void Shutdown() = 0;
  virtual bool Render() = 0;
  virtual inline const float GetFramerate() = 0;
};

}  // namespace app
}  // namespace sensfus

#endif  // APPLICATION_BASE_H