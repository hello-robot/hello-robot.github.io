# Stretch Docs

This repository generates the documentation hosted at [docs.hello-robot.com](https://docs.hello-robot.com).

## Setup

```
python3 -m pip install mkdocs mkdocs-material mkdocstrings==0.17.0 pytkdocs[numpy-style] jinja2=3.0.3
```

## Development

The following command will serve the static website with hot-reloading (i.e. your edits are reflected in real time).

```
python3 -m mkdocs serve
```

## Deploying

Next to this repository, clone [hello-robot/hello-robot.github.io](https://github.com/hello-robot/hello-robot.github.io). The `deploy.sh` script with automatically build the docs, replace the contents of hello-robot.github.io, and push the new docs to master. Github will reflect the changes at [docs.hello-robot.com](https://docs.hello-robot.com).

```
./deploy.sh
```
