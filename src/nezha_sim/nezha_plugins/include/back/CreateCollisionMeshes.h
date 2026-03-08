#pragma once

#include <vector>
#include <gazebo/physics/PhysicsTypes.hh>   // ModelPtr, LinkPtr
#include "asv_wave_sim_gazebo_plugins/CGALTypes.hh"
#include <memory> 
namespace asv
{
using CollisionMesh     = std::vector<Triangle>;
using CollisionMeshList = std::vector<CollisionMesh>;

std::shared_ptr<Mesh> TrianglesToCGALMesh(const CollisionMesh& tris);

/**
 * \brief Convert collision shapes of given links to triangle meshes (world frame).
 *
 * \param links   Links to be converted.
 * \return        One mesh per link. If `links.empty()`, the return value is empty.
 */
[[nodiscard]]
CollisionMeshList CreateCollisionMeshes(const std::vector<gazebo::physics::LinkPtr>& links);
} // namespace asv

