# Base image is created by https://github.com/swift-nav/docker-recipes
FROM 571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-modern-rust:2023-04-25

# Add anything that's specific to this repo's build environment here.
RUN sudo mkdir -p /usr/share/man/man1 \
    && sudo apt-get update \
    && sudo apt-get install --no-install-recommends -y sqlite3 afl++
