# Stretch Documentation Web Server

This repository generates the documentation hosted at [docs.hello-robot.com](https://docs.hello-robot.com).

## Setup for Documentation Development
In order to create a development environment on a 20.04 Ubuntu machine:
```
cd ~/repos
git clone https://github.com/hello-robot/hello-robot.github.io
cd hello-robot.github.io
./install.sh
```
## Branch Structure
The documentation is organized on versioned branches:
```commandline
/repos/hello-robot.github.io$ git branch
  0.1
  0.2
  gh-pages
```
This allows for the development of new documentation under a new version while hosting the stable documentation as default.

All content maintained under the versioned branch. This master branch is for hosting install scripts and this README.md only.

The branch `gh-pages` is a special orphan branch that the documentation build is pushed to using the Mike versioning plug-in for MkDocs. This branch is hosted by default at via GitHub Pages at [docs.hello-robot.com](docs.hello-robot.com),

## Working with Versions
The currently hosted versions that are managed by mike can be found with
```commandline

```
## Editing content
For example
## Deploying

Next to this repository, clone [hello-robot/hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io). The `deploy.sh` script with automatically build the docs, replace the contents of hello-robot.github.io, and push the new docs to master. Github will reflect the changes at [docs.hello-robot.com](https://docs.hello-robot.com).

```
./deploy.sh
```
