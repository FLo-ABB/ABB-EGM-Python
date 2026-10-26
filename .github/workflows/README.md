# Validation and releases

[continuous-deployment.yml](continuous-deployment.yml) separates pull request
validation from publishing.

## Pull requests

Pull requests targeting `main` validate the Python sources and build the source
distribution and wheel. The job has read-only repository permissions and does
not create tags, create GitHub releases, or publish to PyPI.

## Main branch

Pushes to `main` run the same validation. After it succeeds, a separate release
job:

1. Selects the highest version tag and increments its patch version. An existing
   tag in `vMAJOR.MINOR.PATCH` format is required.
2. Updates the version in [setup.py](../../setup.py) and builds the release
   source distribution and wheel.
3. Commits the version bump and creates an annotated tag on that commit. Only
   the tag is pushed; the version-bump commit is not pushed to `main`.
4. Creates a GitHub release with generated release notes.
5. Publishes the built distributions to PyPI.

Release jobs share a concurrency group to prevent simultaneous tag creation.
The release build completes before any tag or release is published.

## Required configuration

- The release job requests `contents: write` for tag and GitHub release creation
  using `GITHUB_TOKEN`.
- The repository secret `PYPI_API_TOKEN2` must contain a PyPI API token authorized
  to publish `ABBRobotEGM`.
