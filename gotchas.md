# Gotchas

- Unused config/schema fields can be planned features, not dead code. Before removing one, check; carry it into the new schema or TODO.md.
- Code that handles a list of wing sections must not assume root first: build_asb_airplane flips downward fins tip-first.
- Do not catch broad exception types to show a friendly message: LinAlgError is a ValueError. Catch a dedicated exception class.
- A design that fails in simulation is a finding about the design. Never loosen test thresholds, soften inputs, auto-trim or change simulation models to make it pass; model changes with big implications are the owner's call, explained first.
- Approval covers exactly what the owner names. A suggestion I offered with several parts is approved only for the parts the reply mentions.
- Cut code by its exact text, never by searching for the next 'def': the next top-level def can be hundreds of lines further (classes in between).
- Before calling a servo/motor output 'already', 'saturated' or 'barely', state it against its range (1000-2000 us, centre 1500, +-500 throw) as a percentage.
