# general structure of workflows

Docker.yml checks for releavant file changes and re-builds required images
Currently images have a dependancy chain of ros -> python -> dev (in the future this might be a tree and can fork)

On top of the dev image then tests are run.
Dev image is also what developers use in their own IDE via devcontainers
https://code.visualstudio.com/docs/devcontainers/containers

# login to github

create personal access token (classic, not fine grained)
https://github.com/settings/tokens

add permissions 
- read:packages scope to download container images and read their metadata.
    
    and optionally,
    
- write:packages scope to download and upload container images and read and write their metadata.
- delete:packages scope to delete container images.

more info @ https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry

login to docker via 

`sh
echo TOKEN | docker login ghcr.io -u GITHUB_USER --password-stdin
`

pull dev image 
`sh
docker pull ghcr.io/dimensionalos/dev
`
