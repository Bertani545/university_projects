#ifndef COLLISIONS_H
#define COLLISIONS_H

#include<iostream>

#include <string>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

//Matrix operations
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>

//Colisions
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>




//https://doc.cgal.org/latest/Polyhedron/index.html

typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Point_3 Point;
typedef K::Segment_3 Segment;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef Tree::Point_and_primitive_id Point_and_primitive_id;


class Object
{
private: 
	
public:
	std::vector<Point> vertices;
	std::vector<std::array<std::size_t, 3>> triangles;
	Polyhedron poly;

	Object(){};
	Object(std::string const &path){

		Assimp::Importer importer2;
		const aiScene* scene = importer2.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals);
		const aiMesh* mesh = scene->mMeshes[0];

		// Create point clouds for model 1 and model 2
		

		// Fill pointCloud1 and pointCloud2 with the transformed vertices
		for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
		    this->vertices.push_back(Point(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z));
		}

		//Get faces
		for(unsigned int i = 0; i < mesh->mNumFaces; i++){
		    this->triangles.push_back({mesh->mFaces[i].mIndices[0], mesh->mFaces[i].mIndices[1], mesh->mFaces[i].mIndices[2]});
		}

		 // Add vertices to the polyhedron
		Polyhedron poly;
		for (unsigned int i = 0; i < triangles.size(); i++) {
		    poly.make_triangle(vertices[triangles[i][0]], vertices[triangles[i][1]], vertices[triangles[i][2]]);
		}

	}

	~Object(){};
	
	void update_state(glm::mat4 M){
		// Transform and Update transformed vertex
		std::vector<Point> new_vertices(vertices.size());

		for (unsigned int i = 0; i < this->vertices.size(); ++i) {
		    glm::vec4 vertex(this->vertices[i].x(), this->vertices[i].y(), this->vertices[i].z(), 1.0f);
		    vertex = M * vertex;
		    new_vertices[i] = {vertex.x, vertex.y, vertex.z};
		}

		//Add vertices to the polyhedron
		poly.clear();
		for (unsigned int i = 0; i < triangles.size(); i++) {
		    poly.make_triangle(new_vertices[triangles[i][0]], new_vertices[triangles[i][1]], new_vertices[triangles[i][2]]);
		}
	}


};


class Scene
{
private:
	std::vector<Polyhedron> poly_meshes;
public:
	
	Scene(){};

	Scene(std::string const &path){
		Assimp::Importer importer;
	    const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals);

	    //Meshes
	    // Extract mesh information for model 2
	    std::vector<aiMesh*> meshes(scene->mNumMeshes);
	    for(unsigned int i = 0; i < scene->mNumMeshes; i++){
	        meshes[i] = scene->mMeshes[i];
	    }

	    //Point Cloud of scene
	    std::vector<std::vector<Point>> vertices_meshes(scene->mNumMeshes);
	    for(unsigned int i = 0; i < scene->mNumMeshes; i++){
	        for (unsigned int j = 0; j < meshes[i]->mNumVertices; ++j) {
	            vertices_meshes[i].push_back(Point(meshes[i]->mVertices[j].x, meshes[i]->mVertices[j].y, meshes[i]->mVertices[j].z));
	        }
	    }
	    


	    //---------------------- NMeshes for collision with triangular faces -------------------------
	    std::vector<std::vector<std::array<std::size_t, 3>>> triangles_meshes(scene->mNumMeshes);  // Replace with triangle indices of mesh2

	    
	    for(unsigned int i = 0; i < scene->mNumMeshes; i++){
	        for(unsigned int j = 0; j < meshes[i]->mNumFaces; j++){
	            triangles_meshes[i].push_back({meshes[i]->mFaces[j].mIndices[0], meshes[i]->mFaces[j].mIndices[1], meshes[i]->mFaces[j].mIndices[2]});
	        }
	    }


	    //Re buld meshes
	    // Construct the polyhedron
	    poly_meshes.resize(scene->mNumMeshes);
	   
	    //Scenery
	    for(unsigned int i = 0; i < scene->mNumMeshes; i++){
	        for(unsigned int j = 0; j < triangles_meshes[i].size(); j++){
	            poly_meshes[i].make_triangle(vertices_meshes[i][triangles_meshes[i][j][0]], vertices_meshes[i][triangles_meshes[i][j][1]], vertices_meshes[i][triangles_meshes[i][j][2]]);
	        }
	    }
	    
	}

	~Scene(){};
	

	bool is_colliding(Object* o){

		//-----------------------------------------------  Object
		//Create object
		Tree tree_obj = Tree(faces(o->poly).first, faces(o->poly).second, o->poly);

		
		//_------------------------------------------ Scene
		//Create scene trees
		Tree tree_scene;
	    for(unsigned int i = 0; i < poly_meshes.size(); i++){
	    	tree_scene.insert(faces(poly_meshes[i]).first, faces(poly_meshes[i]).second, poly_meshes[i]);
	    }


	    return tree_obj.do_intersect(tree_scene);

	}

};
#endif