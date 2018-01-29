#ifndef PX_CG_MESH_HPP
#define PX_CG_MESH_HPP

#include <stdexcept>
#include <string>
#include <memory>
#include <vector>

#include <assimp/mesh.h>
#include <assimp/postprocess.h>

namespace px
{
class Mesh;
class MeshError;
}

class px::Mesh
{
protected:
    class MeshEntry
    {
    public:
        MeshEntry(aiMesh *m);
        ~MeshEntry();
        void render();
        unsigned int vao;
        unsigned int vertex_vbo, index_vbo;
        unsigned int n_indices;

    };
public:
    [[noreturn]]
    void err(std::string const& msg);

    Mesh();

    // triangluate is ensured inside the function
    Mesh(std::string const &model_file,
         unsigned int flags =
//                        aiProcess_Triangulate |
                        aiProcess_CalcTangentSpace |
                        aiProcess_JoinIdenticalVertices |
                        aiProcess_SortByPType|
                        aiProcess_GenNormals);

    void load(std::string const &model_file,
              unsigned int flags =
//                        aiProcess_Triangulate |
                        aiProcess_CalcTangentSpace |
                        aiProcess_JoinIdenticalVertices |
                        aiProcess_SortByPType|
                        aiProcess_GenNormals);
    void init();
    void render();

    std::vector<std::unique_ptr<MeshEntry> > entries;
};

class px::MeshError : public std::exception
{

public:
    MeshError(const std::string &msg, const int code=0)
            : msg(msg), err_code(code)
    {}
    const char *what() const noexcept override
    {
        return msg.data();
    }
    inline int code() const
    {
        return err_code;
    }

protected:
    std::string msg;
    int err_code;
};


#endif
