#include "mesh.hpp"
#include "global.hpp"
#include "shader/base_shader.hpp"

#ifndef NDEBUG
#   include <iostream>
#endif

#include <assimp/Importer.hpp>
#include <assimp/scene.h>

using namespace px;

Mesh::Mesh()
{}

Mesh::Mesh(std::string const &model_file, unsigned int flags)
{
    load(model_file, flags);
}

[[noreturn]]
void Mesh::err(std::string const &msg)
{
    throw MeshError(msg);
}

void Mesh::load(std::string const &model_file, unsigned int flags)
{
    Assimp::Importer importer;
    auto s = importer.ReadFile(model_file, flags | aiProcess_Triangulate);

    if (s == nullptr || s->mFlags == AI_SCENE_FLAGS_INCOMPLETE || !s->mRootNode)
        err("Failed to load model: " + model_file + "\n" + importer.GetErrorString());

#ifndef NDEBUG
    std::cout << "Load Model: " << model_file << "\n";
#endif

    std::vector<std::unique_ptr<MeshEntry> > new_entries;
    new_entries.reserve(s->mNumMeshes);
    entries.swap(new_entries);
    for (decltype(s->mNumMeshes) i = 0; i < s->mNumMeshes; ++i)
    {
        entries.push_back(std::unique_ptr<MeshEntry>(new MeshEntry(s->mMeshes[i])));
    }
}

void Mesh::render()
{
    for (auto &e : entries)
    {
        e->render();
    }
}

Mesh::MeshEntry::~MeshEntry()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vertex_vbo);
    glDeleteBuffers(1, &index_vbo);
}

Mesh::MeshEntry::MeshEntry(aiMesh *m)
    : vao(0), vertex_vbo(0), index_vbo(0), n_indices(0)
{
#define __PUSH_THREE_ZERO         \
    {                           \
        vertices[index++] = 0;  \
        vertices[index++] = 0;  \
        vertices[index++] = 0;  \
    }

    if (!m->HasPositions())
        return;

    auto n_vertices = m->mNumVertices*(m->HasTangentsAndBitangents() ? 11 : 8);
    auto vertices = new float[n_vertices];

    auto index = 0;
    for (decltype(m->mNumVertices) i = 0; i < m->mNumVertices; ++i)
    {
        vertices[index++] = m->mVertices[i].x;
        vertices[index++] = m->mVertices[i].y;
        vertices[index++] = m->mVertices[i].z;


        if (m->HasTextureCoords(0))
        {
            vertices[index++] = m->mTextureCoords[0][i].x;
            vertices[index++] = m->mTextureCoords[0][i].y;
        }
        else
        {
            vertices[index++] = 0;
            vertices[index++] = 0;
        }
        if (m->HasNormals())
        {
            vertices[index++] = m->mNormals[i].x;
            vertices[index++] = m->mNormals[i].y;
            vertices[index++] = m->mNormals[i].z;
        }
        else
        {
            __PUSH_THREE_ZERO
        }
        if (m->HasTangentsAndBitangents())
        {
            vertices[index++] = m->mTangents[i].x;
            vertices[index++] = m->mTangents[i].y;
            vertices[index++] = m->mTangents[i].z;
        }
    }

    n_indices = m->mNumFaces*3;
    auto faces = new unsigned int[n_indices];
    index = 0;
    for (decltype(m->mNumFaces) i = 0; i < m->mNumFaces; ++i)
    {
        faces[index++] = m->mFaces[i].mIndices[0];
        faces[index++] = m->mFaces[i].mIndices[1];
        faces[index++] = m->mFaces[i].mIndices[2];
    }

#ifndef NDEBUG
    std::cout << "Load Mesh: " << m->mNumVertices << " vertices, "
              << sizeof(float)*n_vertices << " Byte, "
              << m->mNumFaces << " faces\n";
#endif

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vertex_vbo);
    glGenBuffers(1, &index_vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_vbo);
    ATTRIB_BIND_HELPER_WITH_TANGENT
    glBufferData(GL_ARRAY_BUFFER, sizeof(float)*n_vertices, vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_vbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, n_indices*sizeof(unsigned int),
                 faces, GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    delete [] vertices;
    delete [] faces;
}

void Mesh::MeshEntry::render()
{
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, n_indices, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}
