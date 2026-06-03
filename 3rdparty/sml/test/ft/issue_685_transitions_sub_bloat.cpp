//
// Copyright (c) 2016-2020 Kris Jusiak (kris at jusiak dot net)
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// Regression test for issue #685:
// transitions_sub<sm<Tsm>>::execute() now skips process_event() when TEvent is
// not in the sub-SM's events_ids_t, avoiding unnecessary template instantiations
// that inflate binary size.
//
// cppcheck-suppress missingIncludeSystem
#include <boost/sml.hpp>

namespace sml = boost::sml;

struct e1 {};
struct e2 {};
struct e3 {};
struct e4 {};  // outer-only event, sub-SM has no handler for this

const auto idle = sml::state<class idle>;
const auto s1 = sml::state<class s1>;
const auto s2 = sml::state<class s2>;

// Sub-SM handles only e1 and e2 (not e3, not e4).
test transitions_sub_bloat_unknown_event_ignored = [] {
  struct sub {
    auto operator()() noexcept {
      using namespace sml;
      return make_transition_table(*idle + event<e1> = s1, s1 + event<e2> = idle);
    }
  };

  struct outer {
    auto operator()() noexcept {
      using namespace sml;
      return make_transition_table(*idle + event<e3> = state<sub>, state<sub> + event<e4> = s2);
    }
  };

  sml::sm<outer> sm{};

  // Outer transitions from idle to sub via e3
  sm.process_event(e3{});
  expect(sm.is(sml::state<sub>));
  expect(sm.is<decltype(sml::state<sub>)>(idle));

  // Sub-SM handles e1 (known event → forwarded)
  sm.process_event(e1{});
  expect(sm.is(sml::state<sub>));
  expect(sm.is<decltype(sml::state<sub>)>(s1));

  // e4 is an outer-only event: sub-SM returns false quickly (no instantiation of sub-SM's process_event<e4>)
  // Outer SM then handles e4 → transitions to s2
  sm.process_event(e4{});
  expect(sm.is(s2));
};

// Verify sub-SM internal events still work correctly after the SFINAE gate
test transitions_sub_bloat_known_events_forwarded = [] {
  struct sub {
    auto operator()() noexcept {
      using namespace sml;
      return make_transition_table(*idle + event<e1> = s1, s1 + event<e2> = idle);
    }
  };

  struct outer {
    auto operator()() noexcept {
      using namespace sml;
      return make_transition_table(*idle + event<e3> = state<sub>, state<sub> + event<e4> = s2);
    }
  };

  sml::sm<outer> sm{};
  sm.process_event(e3{});
  expect(sm.is(sml::state<sub>));

  // e1 and e2 are in sub's events_ids_t → correctly forwarded
  sm.process_event(e1{});
  expect(sm.is<decltype(sml::state<sub>)>(s1));
  sm.process_event(e2{});
  expect(sm.is<decltype(sml::state<sub>)>(idle));
  sm.process_event(e1{});
  expect(sm.is<decltype(sml::state<sub>)>(s1));
};
