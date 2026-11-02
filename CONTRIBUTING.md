# Contributing

## Commits

Conventional Commits: `<type>(<scope>): <summary>`, with a type of `feat`, `fix`, `docs`,
`chore`, `refactor`, `test`, `build`, `ci`, `perf`, `style` or `revert`; a subject of at most 100
characters, no trailing period. Bodies say why.

## AI tools

Work prepared with an AI agent is fine; what enters the history is yours:

- No agent trailers (`Co-Authored-By:`, `Claude-Session:`) in commit messages, pull request
  descriptions or review comments. The one co-author line allowed credits a person, by their
  GitHub noreply address: `Co-authored-by: login <12345+login@users.noreply.github.com>`.
- No AI tool footers ("Generated with ...") and no links to AI chat or agent sessions
  (claude.ai, chatgpt.com): sessions are private, this repository is public.
- Author and committer are you, never an AI agent identity (an anthropic.com or openai.com
  address, Claude, Copilot or another bot).

The `ai-rules` workflow (`scripts/check-ai.py`) checks every pull request and push. Run the same
check on your own commits with the hook:

```sh
ln -sf ../../scripts/commit-msg.py .git/hooks/commit-msg
```

It drops agent trailers from your message and rejects the rest. A line that has to name what the
rules forbid (a test of the rules) carries `ai-rules: allow`.

## Game data

Never commit anything from Jak II's or Ratchet & Clank's discs (models, textures, sounds,
animations, text, executables). The board's model and sounds are made by `tools/make_board.py`
and `tools/make_sounds.py`.
