// cppcheck-suppress missingIncludeSystem
#include <boost/sml.hpp>
// cppcheck-suppress missingIncludeSystem
#include <queue>

namespace sml = boost::sml;
using namespace sml::literals;

struct e1 {};
struct e2 {};

struct AsyncHandle {
  sml::back::process<e2> push{};
};

struct Fired {
  bool value = false;
};

struct SM {
  auto operator()() const {
    using namespace sml;
    return make_transition_table(
        *"s1"_s + event<e1> / [](back::process<e2> p, AsyncHandle &h) { h.push = p; } = "s2"_s,
         "s2"_s + event<e2> / [](Fired &f) { f.value = true; } = X);
  }
};

// Verify that an event pushed via back::process<> after process_event() returns
// is processed when flush_queue() is called.
test flush_queue_processes_async_event = [] {
  AsyncHandle handle;
  Fired fired;
  sml::sm<SM, sml::process_queue<std::queue>> sm{handle, fired};

  sm.process_event(e1{});    // action stores push handle, s1→s2
  expect(!fired.value);

  handle.push(e2{});         // simulates async callback pushing e2 after process_event returns
  expect(!fired.value);      // not yet processed

  sm.flush_queue();          // drain: e2 processed, s2→X
  expect(fired.value);
  expect(sm.is(sml::X));
};

test flush_queue_noop_when_empty = [] {
  AsyncHandle handle;
  Fired fired;
  sml::sm<SM, sml::process_queue<std::queue>> sm{handle, fired};
  sm.flush_queue();          // must not crash or hang on empty queue
  expect(sm.is("s1"_s));
};
