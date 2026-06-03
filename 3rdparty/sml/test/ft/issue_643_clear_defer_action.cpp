// cppcheck-suppress missingIncludeSystem
#include <boost/sml.hpp>
// cppcheck-suppress missingIncludeSystem
#include <deque>
// cppcheck-suppress missingIncludeSystem
#include <queue>

namespace sml = boost::sml;
using namespace sml;

struct e643_go {};
struct e643_ret {};
struct e643_back {};
struct e643_deferred_ev {};

// Test: sml::clear_defer action discards pending deferred events
test clear_defer_action_discards_deferred = [] {
  static int fires = 0;

  struct Sub643 {
    auto operator()() const noexcept {
      return make_transition_table(
         *"a"_s + event<e643_go>           = "b"_s,
          "b"_s + event<e643_deferred_ev> / defer,
          "a"_s + event<e643_deferred_ev> / [] { fires++; },
          "b"_s + event<e643_ret>           = "a"_s
      );
    }
  };
  struct Outer643 {
    auto operator()() const noexcept {
      return make_transition_table(
         *"outer_a"_s + event<e643_go>            = state<Sub643>,
          // clear_defer on the way out so stale events don't survive
          state<Sub643> + event<e643_back> / clear_defer = "outer_a"_s
      );
    }
  };

  sm<Outer643, defer_queue<std::deque>, process_queue<std::queue>> m;

  // Enter sub-SM, defer an event in state "b", then exit with clear_defer
  m.process_event(e643_go{});         // outer_a → Sub643/a
  m.process_event(e643_go{});         // a → b
  m.process_event(e643_deferred_ev{}); // deferred in b
  m.process_event(e643_back{});        // exit Sub643 with clear_defer action

  // Re-enter and do round-trip — the cleared event must not fire
  m.process_event(e643_go{});          // outer_a → Sub643/a
  fires = 0;
  m.process_event(e643_go{});          // a → b
  m.process_event(e643_ret{});         // b → a: no stale deferred event
  expect(fires == 0);
};

// Test: clear_defer is a no-op when defer queue is already empty
test clear_defer_noop_when_empty = [] {
  static int fires = 0;

  struct Inner643 {
    auto operator()() const noexcept {
      return make_transition_table(
         *"x"_s + event<e643_deferred_ev> / [] { fires++; }
      );
    }
  };
  struct Outer643b {
    auto operator()() const noexcept {
      return make_transition_table(
         *"init"_s + event<e643_go>            = state<Inner643>,
          state<Inner643> + event<e643_back> / clear_defer = "init"_s
      );
    }
  };

  sm<Outer643b, defer_queue<std::deque>, process_queue<std::queue>> m;
  m.process_event(e643_go{});
  m.process_event(e643_back{});  // clear_defer on empty queue — must not crash
  fires = 0;
  m.process_event(e643_go{});
  m.process_event(e643_deferred_ev{});  // normal event fires in x state
  expect(fires == 1);
};
