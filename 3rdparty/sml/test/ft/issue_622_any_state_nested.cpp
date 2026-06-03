//
// Copyright (c) 2016-2020 Kris Jusiak (kris at jusiak dot net)
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Issue #622: `any` wildcard in outer SM is never checked when the outer SM is
//   in a composable sub-state (struct with public operator()) that itself
//   handles the triggering event — the sub-SM's dispatch short-circuits the
//   outer wildcard even after the sub-SM terminates.
//
// cppcheck-suppress missingIncludeSystem
#include <boost/sml.hpp>

namespace sml = boost::sml;

struct e1 {};
struct e2 {};

struct state_init {};
struct state_b {};
struct state_inner {};

// struct → public operator() → composable → treated as sm<sub_a>
struct sub_a {
  auto operator()() const noexcept {
    using namespace sml;
    return make_transition_table(
        *state<state_inner> + event<e1> = X  // sub-SM handles e1 and terminates
    );
  }
};

// Outer SM: init -e2-> sub_a; any -e1-> state_b
struct outer {
  auto operator()() const noexcept {
    using namespace sml;
    const auto any = state<_>;
    // clang-format off
    return make_transition_table(
        *state<state_init> + event<e2> = state<sub_a>
      , any               + event<e1> = state<state_b>  // wildcard: anywhere → b on e1
    );
    // clang-format on
  }
};

// After e2: outer SM is in sub_a (sub-SM starts in state_inner).
// After e1: sub-SM handles e1 (state_inner→X, terminates).
//           Outer wildcard any+e1→state_b must then fire.
// Expected: outer SM is in state_b.
test any_state_fires_after_nested_sub_sm_terminates = [] {
  sml::sm<outer> sm;

  expect(sm.is(sml::state<state_init>));

  sm.process_event(e2{});
  expect(sm.is(sml::state<sub_a>));
  expect(sm.is<decltype(sml::state<sub_a>)>(sml::state<state_inner>));

  sm.process_event(e1{});
  expect(sm.is(sml::state<state_b>));
};

// When the outer SM is in a regular (non-composable) state, the wildcard
// already works correctly.  Verify this baseline is not regressed.
struct state_leaf {};
struct outer_leaf_only {
  auto operator()() const noexcept {
    using namespace sml;
    const auto any = state<_>;
    // clang-format off
    return make_transition_table(
        *state<state_init> + event<e2> = state<state_leaf>
      , any                + event<e1> = state<state_b>
    );
    // clang-format on
  }
};

test any_state_leaf_baseline = [] {
  sml::sm<outer_leaf_only> sm;

  sm.process_event(e2{});
  expect(sm.is(sml::state<state_leaf>));

  sm.process_event(e1{});
  expect(sm.is(sml::state<state_b>));
};
