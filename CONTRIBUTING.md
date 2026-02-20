# Contributing

## Branching Strategy
- `main`: stable, protected
- `feature/<name>`: new features
- `fix/<name>`: bug fixes

## Development Workflow
1. Create a feature branch from `main`.
2. Implement changes.
3. Ensure:
   - `colcon build --base-paths src` succeeds
   - `colcon test --base-paths src` passes
   - No lint errors
4. Open a Pull Request.

## Code Style
- C++: `ament_cmake` + `clang-format`
- Python: `black` + PEP8
- Follow `.editorconfig` rules

## Commit Messages
Use concise, imperative style:
- Add control node
- Fix parameter parsing
- Refactor launch structure

## Testing
All new features must include:
- Unit tests when applicable
- Launch verification
- No regression failures

## CI
Pull requests must pass CI before merge.
