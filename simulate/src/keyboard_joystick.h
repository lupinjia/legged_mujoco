#pragma once

#include <atomic>

#include <GLFW/glfw3.h>

#include <legged_sdk/common/legged_joystick.hpp>

#include "glfw_adapter.h"

// Keyboard input for the legged bridge.
//
// The MuJoCo viewer only records the *last* key event in mjuiState, so it cannot
// tell which keys are currently held. KeyboardGlfwAdapter derives from
// mj::GlfwAdapter and overrides OnKey() to keep a per-key "held" state, while
// still forwarding every event to the base adapter so the viewer's own
// shortcuts (Esc, Enter, F-keys, ...) keep working. The adapter instance is
// created in main.cc and passed to the bridge thread; KeyboardJoystick polls it
// from the real-time bridge loop.

class KeyboardGlfwAdapter : public mujoco::GlfwAdapter
{
public:
  void OnKey(int key, int scancode, int act) override
  {
    if (key >= 0 && key <= GLFW_KEY_LAST) {
      keys_[key].store(act != GLFW_RELEASE, std::memory_order_relaxed);
    }
    // Keep the viewer's key handling (camera tracking, stepping, ...) intact.
    mujoco::GlfwAdapter::OnKey(key, scancode, act);
  }

  bool key(int k) const
  {
    return k >= 0 && k <= GLFW_KEY_LAST && keys_[k].load(std::memory_order_relaxed);
  }

private:
  // GLFW key states are written from the render thread and read from the bridge
  // thread; atomics keep that well-defined.
  std::atomic<bool> keys_[GLFW_KEY_LAST + 1]{};
};

// Keyboard-driven legged joystick. Adjust the mapping in update() to taste.
//
//   left stick  : W / S -> ly (forward/back), A / D -> lx (left/right)
//   right stick : I / K -> ry,                J / L -> rx
//   d-pad       : arrow keys (note: Left/Right are also MuJoCo step shortcuts)
//   buttons     : Space=A, Left Shift=B, Q=X, E=Y, 1=L1, 2=R1, 3=start, 4=select
class KeyboardJoystick : public legged::common::LeggedJoystick
{
public:
  explicit KeyboardJoystick(const KeyboardGlfwAdapter *adapter) : adapter_(adapter) {}

  void update() override
  {
    if (!adapter_) {
      return;
    }
    auto axis = [this](int neg, int pos) {
      return static_cast<float>(adapter_->key(pos)) - static_cast<float>(adapter_->key(neg));
    };

    lx(axis(GLFW_KEY_A, GLFW_KEY_D));
    ly(axis(GLFW_KEY_S, GLFW_KEY_W));
    rx(axis(GLFW_KEY_J, GLFW_KEY_L));
    ry(axis(GLFW_KEY_K, GLFW_KEY_I));

    up(adapter_->key(GLFW_KEY_UP));
    down(adapter_->key(GLFW_KEY_DOWN));
    left(adapter_->key(GLFW_KEY_LEFT));
    right(adapter_->key(GLFW_KEY_RIGHT));

    A(adapter_->key(GLFW_KEY_SPACE));
    B(adapter_->key(GLFW_KEY_LEFT_SHIFT));
    X(adapter_->key(GLFW_KEY_Q));
    Y(adapter_->key(GLFW_KEY_E));
    L1(adapter_->key(GLFW_KEY_1));
    R1(adapter_->key(GLFW_KEY_2));
    start(adapter_->key(GLFW_KEY_3));
    select(adapter_->key(GLFW_KEY_4));
  }

private:
  const KeyboardGlfwAdapter *adapter_;
};
