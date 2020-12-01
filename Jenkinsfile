#!groovy

/**
 * This Jenkinsfile will only work in a Swift Navigation build/CI environment, as it uses
 * non-public docker images and pipeline libraries.
 */

// Use 'ci-jenkins@somebranch' to pull shared lib from a different branch than the default.
// Default is configured in Jenkins and should be from "stable" tag.
@Library("ci-jenkins") import com.swiftnav.ci.*

def context = new Context(context: this)
context.setRepo("gnss-converters")
def builder = context.getBuilder()

/**
 * - Mount the ccache to speed up builds
 * - Mount the refrepo to keep git operations functional on a repo that uses ref-repo during clone
 **/
String dockerMountArgs = "-v /mnt/efs/ccache:/home/jenkins/.ccache -v /mnt/efs/refrepo:/mnt/efs/refrepo"

pipeline {
    // Override agent in each stage to make sure we don't share containers among stages.
    agent any
    options {
        // Make sure job aborts after 2 hours if hanging.
        timeout(time: 2, unit: 'HOURS')
        timestamps()
        // Keep builds for 7 days.
        buildDiscarder(logRotator(daysToKeepStr: '3'))
    }

    // Overwrite in stages that need clang
    environment {
        // Default compiler. Override in each stage as needed.
        CC='gcc-6'
        CXX='g++-6'
        COMPILER='gcc-6'

        // Default parallelism for make
        MAKEJ='4'

        // Since ~/.ccache is mounted from a shared NFS disk, make sure the temp
        // dir is local to the container.
        CCACHE_TEMPDIR='/tmp/ccache_tmp'
    }

    stages {
        stage('Build') {
            parallel {
                stage('Haskell Stack') {
                    agent {
                        dockerfile {
                            filename "Dockerfile_Haskell"
                            args dockerMountArgs
                        }
                    }
                    steps {
                            sh("""#!/bin/bash -ex
                                cd haskell && stack build --test && cd ../
                                """)
                    }
                }
                stage('Build c') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            builder.cmake(workDir: "c", cmakeAddArgs: "-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -Dnov2sbp_BUILD=true")
                            builder.make(workDir: "c/build")
                            builder.make(workDir: "c/build", target: "clang-format-all")
                        }
                        /** Run clang-format.
                         *  If the resulting 'git diff' is non-empty, then it found something,
                         *  so error out and display the diff.
                         */
                        sh '''#!/bin/bash -ex
                            git --no-pager diff --name-only HEAD > /tmp/clang-format-diff
                            if [ -s "/tmp/clang-format-diff" ]; then
                                echo "clang-format warning found"
                                git --no-pager diff
                                exit 1
                            fi
                            '''

                        sh '''#!/bin/bash -ex
                            (cd c && cd build && make clang-tidy-all)
                            if [ -e "c/fixes.yaml" ]; then
                                echo "clang-tidy warning found"
                                exit 1
                            fi
                            '''
                    }
                    post {
                        always {
                            archiveArtifacts(artifacts: 'c/fixes.yaml', allowEmptyArchive: true)
                        }
                    }
                }
            }
        }
    }
}
