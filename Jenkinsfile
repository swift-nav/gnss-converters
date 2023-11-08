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

    stages {
        stage('Build') {
            parallel {
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
			script {
			    builder.make(workDir: "c/build", target: "do-all-tests")
			}
                    }
                    post {
                        always {
                            archiveArtifacts(artifacts: 'c/fixes.yaml', allowEmptyArchive: true)
                        }
                    }
                }
                stage('Fuzz Test Checker') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    environment {
                        CFLAGS='-fsanitize=address'
                        CXXFLAGS='-fsanitize=address'
                        LDFLAGS='-fsanitize=address'
                    }
                    steps {
                        gitPrep()
                        fetchTestData()
                        script {
                            builder.cmake(workDir: 'c', cmakeAddArgs: '-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -DI_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION=true')
                            builder.make(workDir: 'c/build', target: 'rerun-known-failures-ubx2sbp')
                            builder.make(workDir: 'c/build', target: 'rerun-known-failures-rtcm3tosbp')
                        }
                    }
                }
            }
        }
    }
    post {
        success {
            script {
                def automatedPr = new AutomatedPR(context: context)
                automatedPr.merge()
            }
        }

        failure {
            script {
                def automatedPr = new AutomatedPR(context: context)
                automatedPr.alertSlack()
            }
        }

        always {
            script {
                context.slackNotify(channel: '#positioning-dev')
                context.slackNotify(channel: '#release', branches: ['.*v.*-release'])
            }
        }

        cleanup {
            cleanWs()
        }
    }
}

/**
 * Retrieve test data submodule (it's large and not needed for all stages, so
 * don't fetch it unless needed).
 * @param args
 * @return
 */
def fetchTestData(Map args = [:]) {
    sh 'git submodule update --init --checkout --recursive c/afl/findings'
}
