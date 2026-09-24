#pragma once

#include <grpc++/client_context.h>
#include <memory>
#include <mutex>

namespace blickfeld::ros_interop::qb2 {

// One RPC worker owns creation/reset and uses get(). Other threads may cancel.
// Destroy the reader before resetting its context. A permanent stop also
// cancels contexts created after shutdown has begun (e.g. during connection).
class CancellableContext {
 public:
  void reset(std::unique_ptr<grpc::ClientContext> context = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    context_ = std::move(context);
    if (stopped_ && context_) context_->TryCancel();
  }

  grpc::ClientContext* get() const { return context_.get(); }

  void cancel(bool permanently = false) {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = stopped_ || permanently;
    if (context_) context_->TryCancel();
  }

 private:
  std::mutex mutex_;
  std::unique_ptr<grpc::ClientContext> context_;
  bool stopped_ = false;
};

}  // namespace blickfeld::ros_interop::qb2
