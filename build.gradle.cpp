plugins {
    id "cpp"
    id "google-test-test-suite"
    id "edu.wpi.first.GradleRIO" version "2025.3.2"
}

// Define my targets (RoboRIO) and artifacts (deployable files)
deploy {
    targets {
        roborio(getTargetTypeClass('RoboRIO')) {
            team = project.frc.getTeamNumber()
            debug = project.frc.getDebugOrDefault(false)

            artifacts {
                frcCpp(getArtifactTypeClass('FRCNativeArtifact')) {
                }

                // Static files artifact
                frcStaticFileDeploy(getArtifactTypeClass('FileTreeArtifact')) {
                    files = project.fileTree('src/main/deploy')
                    directory = '/home/lvuser/deploy'
                    deleteOldFiles = true
                }
            }
        }
    }
}

def deployArtifact = deploy.targets.roborio.artifacts.frcCpp

// Set to true to use debug for JNI.
wpi.cpp.debugJni = false

// Set this to true to enable desktop support.
def includeDesktopSupport = true

model {
    components {
        frcUserProgram(NativeExecutableSpec) {
            targetPlatform wpi.platforms.roborio
            if (includeDesktopSupport) {
                targetPlatform wpi.platforms.desktop
            }

            sources.cpp {
                source {
                    srcDir 'src/main/cpp'
                    include '**/*.cpp', '**/*.cc'
                }
                exportedHeaders {
                    srcDir 'src/main/include'
                }
            }

            // Defining my dependencies. In this case, WPILib (+ friends), and vendor libraries.
            wpi.cpp.vendor.cpp(it)
            wpi.cpp.deps.wpilib(it)
        }
    }
    testSuites {
        frcUserProgramTest(GoogleTestTestSuiteSpec) {
            testing $.components.frcUserProgram

            sources.cpp {
                source {
                    srcDir 'src/test/cpp'
                    include '**/*.cpp'
                }
            }

            wpi.cpp.vendor.cpp(it)
            wpi.cpp.deps.wpilib(it)
            wpi.cpp.deps.googleTest(it)
        }
    }
}

// Simulation configuration (e.g. environment variables).
wpi.sim.addGui().defaultEnabled = true
wpi.sim.addDriverstation()

deployArtifact.artifact = frcUserProgram
