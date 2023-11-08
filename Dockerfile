# Base image is created by https://github.com/swift-nav/docker-recipes
FROM 571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-rust:2022-05-25

# Add anything that's specific to this repo's build environment here.
RUN sudo mkdir -p /usr/share/man/man1
RUN sudo apt-get update && sudo apt-get install -y sqlite3 afl++