#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/simple_idt.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "polyscope/polyscope.h"
#include "polyscope/volume_mesh.h"

#include "args/args.hxx"
#include "imgui.h"

#include "utils.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

// Polyscope visualization handle, to quickly add data to the surface
polyscope::VolumeMesh* psMesh;
polyscope::VolumeMeshVertexScalarQuantity* psScalar;

// == Geometry data
double wx = 1, wy = 1, wz = 1;
size_t N = 10;
Eigen::MatrixXd gridPoints;
Eigen::MatrixXi gridHexes;

void initGeom() {
    gridPoints = Eigen::MatrixXd(N * N * N, 3);
    gridHexes  = Eigen::MatrixXi((N - 1) * (N - 1) * (N - 1), 8);

    double dx = wx / (N - 1);
    double dy = wy / (N - 1);
    double dz = wz / (N - 1);
    for (size_t iX = 0; iX < N; iX++) {
        for (size_t iY = 0; iY < N; iY++) {
            for (size_t iZ = 0; iZ < N; iZ++) {
                double x = dx * iX - wx / 2.;
                double y = dy * iY - wx / 2.;
                double z = dz * iZ - wx / 2.;

                size_t iV         = N * N * iX + N * iY + iZ;
                gridPoints(iV, 0) = x;
                gridPoints(iV, 1) = y;
                gridPoints(iV, 2) = z;

                if (iX + 1 >= N || iY + 1 >= N || iZ + 1 >= N) continue;

                size_t iC        = (N - 1) * (N - 1) * iX + (N - 1) * iY + iZ;
                gridHexes(iC, 0) = N * N * (iX + 0) + N * (iY + 0) + (iZ + 0);
                gridHexes(iC, 1) = N * N * (iX + 1) + N * (iY + 0) + (iZ + 0);
                gridHexes(iC, 2) = N * N * (iX + 1) + N * (iY + 1) + (iZ + 0);
                gridHexes(iC, 3) = N * N * (iX + 0) + N * (iY + 1) + (iZ + 0);
                gridHexes(iC, 4) = N * N * (iX + 0) + N * (iY + 0) + (iZ + 1);
                gridHexes(iC, 5) = N * N * (iX + 1) + N * (iY + 0) + (iZ + 1);
                gridHexes(iC, 6) = N * N * (iX + 1) + N * (iY + 1) + (iZ + 1);
                gridHexes(iC, 7) = N * N * (iX + 0) + N * (iY + 1) + (iZ + 1);
            }
        }
    }
}

// A user-defined callback, for creating control panels (etc)
// Use ImGUI commands to build whatever you want here, see
// https://github.com/ocornut/imgui/blob/master/imgui.h
void myCallback() {
    static bool showLevelSet = true;
    if (!showLevelSet && ImGui::Button("Show level set")) {
        showLevelSet = true;
        psScalar->setEnabledLevelSet(showLevelSet);
    } else if (showLevelSet && ImGui::Button("Hide level set")) {
        showLevelSet = false;
        psScalar->setEnabledLevelSet(showLevelSet);
    }

    static float val = 0.3f;
    if (ImGui::SliderFloat("Level set val", &val, 0.0f, 0.5f)) {
        psScalar->setLevelSetValue(val);
    }
}

int main(int argc, char** argv) {

    // Configure the argument parser
    args::ArgumentParser parser("Geometry program");

    // Parse args
    try {
        parser.ParseCLI(argc, argv);
    } catch (args::Help) {
        std::cout << parser;
        return 0;
    } catch (args::ParseError e) {
        std::cerr << e.what() << std::endl;
        std::cerr << parser;
        return 1;
    }

    initGeom();

    // Initialize polyscope
    polyscope::init();

    // Set the callback function
    polyscope::state::userCallback = myCallback;

    // Register the mesh with polyscope
    psMesh = polyscope::registerVolumeMesh("grid", gridPoints, gridHexes);

    Eigen::VectorXd f(gridPoints.rows());
    for (Eigen::Index iV = 0; iV < gridPoints.rows(); iV++) {
        f(iV) = gridPoints.row(iV).norm();
    }

    psScalar = psMesh->addVertexScalarQuantity("f", f);
    psScalar->setLevelSetValue(0.3);
    psScalar->setEnabledLevelSet(true);

    // Give control to the polyscope gui
    polyscope::show();

    return EXIT_SUCCESS;
}
