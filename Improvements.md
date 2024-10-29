# Improvements

## General

- Runbooks: 
	- Instructions on what to run in what order, for a given task
	- Common errors: include workarounds for common errors and possible solutions
	- Debug: Multiple ways of testing if each part of the pipeline is working

## Integration

- Use enums for task managers instead of dicts, or combine with dicts.
- Let areas define their own possible actions
- Launch terminals with predefined config easier
- Add linter and perform github check

## HRI

- Move non-sensitive data from .env to .yaml, to avoid having to rebuild docker for refresh. Parse yaml data and add meaningful comments.
- Structured outputs when interacting with LLMs.
