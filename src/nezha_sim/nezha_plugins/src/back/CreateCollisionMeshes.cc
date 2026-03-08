#include "CreateCollisionMeshes.h"
#include <gazebo/common/MeshManager.hh>
#include <gazebo/common/SubMesh.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/MeshShape.hh>
#include <ignition/math/Pose3.hh>

#include "asv_wave_sim_gazebo_plugins/Convert.hh"

namespace asv
{

std::shared_ptr<Mesh> TrianglesToCGALMesh(const CollisionMesh& tris)
{
  auto surf = std::make_shared<Mesh>();
  surf->clear();

  for (const auto& tri : tris)
  {
    Mesh::Vertex_index vi[3];
    for (int k = 0; k < 3; ++k)
      vi[k] = surf->add_vertex(tri.vertex(k));

    surf->add_face(vi[0], vi[1], vi[2]);
  }
  return surf;
}
namespace
{
/* 把 ignition::math::Vector3d 转成 Point3 */
inline Point3 ToPoint3(const ignition::math::Vector3d& v)
{
  return {v.X(), v.Y(), v.Z()};
}
} // anonymous namespace

 CollisionMeshList CreateCollisionMeshes(const std::vector<gazebo::physics::LinkPtr>& links)
{
  using namespace gazebo;

  CollisionMeshList meshes;
   meshes.reserve(links.size());

  auto* meshMgr = common::MeshManager::Instance();

  //---------------------------------------------
  // 遍历每个 link
  //---------------------------------------------
  for (const auto& link : links)
  {
CollisionMesh triangles;
    if (!link)
    {
      meshes.emplace_back(std::move(triangles));
      continue;
    }

    //-------------------------------------------
    // 遍历 Collision
    //-------------------------------------------
    for (const auto& collision : link->GetCollisions())
    {
      if (!collision) continue;
      auto shape = collision->GetShape();
      if (!shape) continue;

      auto worldPose = link->WorldPose() + collision->RelativePose();

      //--------------------------------------------------
      // 1. MeshShape
      //--------------------------------------------------
      if (shape->HasType(physics::Base::MESH_SHAPE))
      {
        auto meshShape = boost::dynamic_pointer_cast<physics::MeshShape>(shape);
        if (!meshShape) continue;

        if (auto* mesh = meshMgr->GetMesh(meshShape->GetMeshURI()); mesh)
        {
          const auto& scale = meshShape->Scale();

          for (unsigned int si = 0; si < mesh->GetSubMeshCount(); ++si)
          {
            const auto* sub = mesh->GetSubMesh(si);
            if (!sub) continue;

            for (size_t idx = 0; idx + 2 < sub->GetIndexCount(); idx += 3)
            {
              ignition::math::Vector3d v[3] =
              {
                sub->Vertex(sub->GetIndex(idx    )),
                sub->Vertex(sub->GetIndex(idx + 1)),
                sub->Vertex(sub->GetIndex(idx + 2))
              };

              for (auto& p : v)
                p = worldPose * (p * scale);

              triangles.push_back({ToPoint3(v[0]), ToPoint3(v[1]), ToPoint3(v[2])});
            }
          }
        }
      }
      //--------------------------------------------------
      // 2. Primitive (box / cylinder / sphere)
      //--------------------------------------------------
      else if (shape->HasType(physics::Base::BOX_SHAPE) ||
               shape->HasType(physics::Base::CYLINDER_SHAPE) ||
               shape->HasType(physics::Base::SPHERE_SHAPE))
      {
        std::vector<Triangle> prim;
        Convert::ShapeToTriangles(*shape, prim);

        for (auto& tri : prim)
          for (auto& p : tri)
          {
            ignition::math::Vector3d v(p.x(), p.y(), p.z());
            p = ToPoint3(worldPose * v);
          }

        // 挪入 triangles
        std::move(prim.begin(), prim.end(), std::back_inserter(triangles));
      }
    } // for collision

    meshes.emplace_back(std::move(triangles));
  } // for link

  return meshes;
}
} // namespace asv

