#include <rigidbodies/render/gl_render_device.hpp>

#include <rigidbodies/core/log.hpp>
#include <rigidbodies/render/draw_compiler.hpp>
#include <rigidbodies/render/font_atlas.hpp>

#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include <SDL3/SDL_opengl_glext.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace rigidbodies::render
{
    namespace
    {
        struct GlFunctions
        {
#define RB_GL_FUNCTIONS(X)                                                                                 \
    X(const GLubyte*, GetString, (GLenum))                                                                 \
    X(void, GetIntegerv, (GLenum, GLint*))                                                                 \
    X(GLenum, GetError, ())                                                                                \
    X(void, Enable, (GLenum))                                                                              \
    X(void, Disable, (GLenum))                                                                             \
    X(void, BlendFunc, (GLenum, GLenum))                                                                   \
    X(void, Viewport, (GLint, GLint, GLsizei, GLsizei))                                                    \
    X(void, Scissor, (GLint, GLint, GLsizei, GLsizei))                                                     \
    X(void, ClearColor, (GLfloat, GLfloat, GLfloat, GLfloat))                                              \
    X(void, Clear, (GLbitfield))                                                                           \
    X(void, GenTextures, (GLsizei, GLuint*))                                                               \
    X(void, DeleteTextures, (GLsizei, const GLuint*))                                                      \
    X(void, BindTexture, (GLenum, GLuint))                                                                 \
    X(void, TexParameteri, (GLenum, GLenum, GLint))                                                        \
    X(void, TexImage2D, (GLenum, GLint, GLint, GLsizei, GLsizei, GLint, GLenum, GLenum, const void*))      \
    X(void, PixelStorei, (GLenum, GLint))                                                                  \
    X(void, ReadPixels, (GLint, GLint, GLsizei, GLsizei, GLenum, GLenum, void*))                           \
    X(void, DrawElements, (GLenum, GLsizei, GLenum, const void*))                                          \
    X(void, DrawElementsInstanced, (GLenum, GLsizei, GLenum, const void*, GLsizei))                        \
    X(void, ActiveTexture, (GLenum))                                                                       \
    X(GLuint, CreateShader, (GLenum))                                                                      \
    X(void, ShaderSource, (GLuint, GLsizei, const GLchar* const*, const GLint*))                           \
    X(void, CompileShader, (GLuint))                                                                       \
    X(void, GetShaderiv, (GLuint, GLenum, GLint*))                                                         \
    X(void, GetShaderInfoLog, (GLuint, GLsizei, GLsizei*, GLchar*))                                        \
    X(void, DeleteShader, (GLuint))                                                                        \
    X(GLuint, CreateProgram, ())                                                                           \
    X(void, AttachShader, (GLuint, GLuint))                                                                \
    X(void, LinkProgram, (GLuint))                                                                         \
    X(void, GetProgramiv, (GLuint, GLenum, GLint*))                                                        \
    X(void, GetProgramInfoLog, (GLuint, GLsizei, GLsizei*, GLchar*))                                       \
    X(void, DeleteProgram, (GLuint))                                                                       \
    X(void, UseProgram, (GLuint))                                                                          \
    X(GLint, GetUniformLocation, (GLuint, const GLchar*))                                                  \
    X(void, Uniform1i, (GLint, GLint))                                                                     \
    X(void, Uniform2f, (GLint, GLfloat, GLfloat))                                                          \
    X(void, GenVertexArrays, (GLsizei, GLuint*))                                                           \
    X(void, DeleteVertexArrays, (GLsizei, const GLuint*))                                                  \
    X(void, BindVertexArray, (GLuint))                                                                     \
    X(void, GenBuffers, (GLsizei, GLuint*))                                                                \
    X(void, DeleteBuffers, (GLsizei, const GLuint*))                                                       \
    X(void, BindBuffer, (GLenum, GLuint))                                                                  \
    X(void, BufferData, (GLenum, GLsizeiptr, const void*, GLenum))                                         \
    X(void, EnableVertexAttribArray, (GLuint))                                                             \
    X(void, VertexAttribPointer, (GLuint, GLint, GLenum, GLboolean, GLsizei, const void*))                 \
    X(void, VertexAttribDivisor, (GLuint, GLuint))                                                         \
    X(void, GenFramebuffers, (GLsizei, GLuint*))                                                           \
    X(void, DeleteFramebuffers, (GLsizei, const GLuint*))                                                  \
    X(void, BindFramebuffer, (GLenum, GLuint))                                                             \
    X(void, FramebufferTexture2D, (GLenum, GLenum, GLenum, GLuint, GLint))                                 \
    X(GLenum, CheckFramebufferStatus, (GLenum))                                                            \
    X(void, BlitFramebuffer, (GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLint, GLbitfield, GLenum)) \
    X(void, GenRenderbuffers, (GLsizei, GLuint*))                                                          \
    X(void, DeleteRenderbuffers, (GLsizei, const GLuint*))                                                 \
    X(void, BindRenderbuffer, (GLenum, GLuint))                                                            \
    X(void, RenderbufferStorageMultisample, (GLenum, GLsizei, GLenum, GLsizei, GLsizei))                   \
    X(void, GetRenderbufferParameteriv, (GLenum, GLenum, GLint*))                                          \
    X(void, FramebufferRenderbuffer, (GLenum, GLenum, GLenum, GLuint))
// These macro arguments are type/declarator tokens, not expressions.
// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define RB_GL_DECLARE(result, name, arguments) result(APIENTRY* name) arguments {};
            RB_GL_FUNCTIONS(RB_GL_DECLARE)
#undef RB_GL_DECLARE
            bool load(std::string& error)
            {
#define RB_GL_LOAD(result, name, arguments)                                       \
    (name) = reinterpret_cast<decltype(name)>(SDL_GL_GetProcAddress("gl" #name)); \
    if (!(name))                                                                  \
    {                                                                             \
        error = "OpenGL 3.3 entry point is unavailable: gl" #name;                \
        return false;                                                             \
    }
                RB_GL_FUNCTIONS(RB_GL_LOAD)
#undef RB_GL_LOAD
                return true;
            }
#undef RB_GL_FUNCTIONS
        };

        constexpr const char* vertex_shader = R"(#version 330 core
layout(location=0) in vec2 position;
layout(location=1) in vec2 texcoord;
layout(location=2) in vec4 color;
layout(location=3) in vec4 translation_scale;
layout(location=4) in vec2 rotation_cs;
layout(location=5) in vec4 tint;
uniform vec2 viewport_size;
out vec2 uv;
out vec4 vertex_color;
void main() {
    vec2 p = position * translation_scale.zw;
    p = vec2(rotation_cs.x * p.x - rotation_cs.y * p.y,
             rotation_cs.y * p.x + rotation_cs.x * p.y) + translation_scale.xy;
    gl_Position = vec4(p.x * 2.0 / viewport_size.x - 1.0,
                       1.0 - p.y * 2.0 / viewport_size.y, 0.0, 1.0);
    uv = texcoord;
    vertex_color = color * tint;
})";
        // A negative texture coordinate marks untextured geometry, which may then share a draw
        // call with whatever texture its neighbours use.
        constexpr const char* fragment_shader = R"(#version 330 core
uniform sampler2D image;
in vec2 uv;
in vec4 vertex_color;
out vec4 result;
void main() {
    vec4 sampled = texture(image, uv);
    result = (uv.x < -0.5 ? vec4(1.0) : sampled) * vertex_color;
}
)";

        struct PackedVertex
        {
            float x, y, u, v, r, g, b, a;
        };
        struct PackedInstance
        {
            float x, y, sx, sy, c, s, r, g, b, a;
        };
    } // namespace

    struct GlRenderDevice::Impl
    {
        struct Target
        {
            ViewportSize size;
            int samples { 1 };
            GLuint draw {}, resolve {}, texture {}, color_buffer {};
        };
        struct Texture
        {
            std::weak_ptr<const TexturePixels> pixels;
            GLuint name {};
        };
        SDL_Window* window {};
        SDL_GLContext context {};
        GlFunctions gl;
        bool functions_ready {};
        std::string error, name { "OpenGL 3.3 core" };
        FontAtlas font;
        GLuint program {}, vao {}, vertices {}, indices {}, instances {}, white {};
        GLint viewport_uniform {}, maximum_samples { 1 }, maximum_size { 1 };
        int requested_samples { 4 };
        Target screen;
        std::unordered_map<std::uint64_t, Target> targets;
        std::unordered_map<const TexturePixels*, Texture> textures;
        std::uint64_t next_target { 1 }, active_target {};

        ~Impl()
        {
            if (context)
                SDL_GL_MakeCurrent(window, context);
            if (functions_ready)
            {
                for (auto& [id, target] : targets)
                {
                    (void)id;
                    release(target);
                }
                release(screen);
                for (const auto& [pixels, texture] : textures)
                {
                    (void)pixels;
                    gl.DeleteTextures(1, &texture.name);
                }
                if (white)
                    gl.DeleteTextures(1, &white);
                if (program)
                    gl.DeleteProgram(program);
                if (vao)
                    gl.DeleteVertexArrays(1, &vao);
                const GLuint buffers[] { vertices, indices, instances };
                gl.DeleteBuffers(3, buffers);
            }
            if (context)
                SDL_GL_DestroyContext(context);
        }

        void release(Target& target)
        {
            if (target.draw && target.draw != target.resolve)
                gl.DeleteFramebuffers(1, &target.draw);
            if (target.resolve)
                gl.DeleteFramebuffers(1, &target.resolve);
            if (target.texture)
                gl.DeleteTextures(1, &target.texture);
            if (target.color_buffer)
                gl.DeleteRenderbuffers(1, &target.color_buffer);
            target = {};
        }

        GLuint shader(GLenum type, const char* source)
        {
            const auto result = gl.CreateShader(type);
            gl.ShaderSource(result, 1, &source, nullptr);
            gl.CompileShader(result);
            GLint status {};
            gl.GetShaderiv(result, GL_COMPILE_STATUS, &status);
            if (!status)
            {
                std::array<char, 4096> log {};
                gl.GetShaderInfoLog(result, static_cast<GLsizei>(log.size()), nullptr, log.data());
                error = std::string { "OpenGL shader compilation failed: " } + log.data();
                gl.DeleteShader(result);
                return 0;
            }
            return result;
        }

        bool initialize()
        {
            functions_ready = gl.load(error);
            if (!functions_ready)
                return false;
            GLint major {}, minor {};
            gl.GetIntegerv(GL_MAJOR_VERSION, &major);
            gl.GetIntegerv(GL_MINOR_VERSION, &minor);
            if (major < 3 || (major == 3 && minor < 3))
            {
                error = "OpenGL 3.3 or newer is required";
                return false;
            }
            gl.GetIntegerv(GL_MAX_SAMPLES, &maximum_samples);
            maximum_samples = std::max(1, maximum_samples);
            gl.GetIntegerv(GL_MAX_TEXTURE_SIZE, &maximum_size);
            GLint buffer_limit {};
            gl.GetIntegerv(GL_MAX_RENDERBUFFER_SIZE, &buffer_limit);
            maximum_size = std::min(maximum_size, buffer_limit);
            const auto vs = shader(GL_VERTEX_SHADER, vertex_shader);
            const auto fs = shader(GL_FRAGMENT_SHADER, fragment_shader);
            if (!vs || !fs)
            {
                if (vs)
                    gl.DeleteShader(vs);
                if (fs)
                    gl.DeleteShader(fs);
                return false;
            }
            program = gl.CreateProgram();
            gl.AttachShader(program, vs);
            gl.AttachShader(program, fs);
            gl.LinkProgram(program);
            gl.DeleteShader(vs);
            gl.DeleteShader(fs);
            GLint linked {};
            gl.GetProgramiv(program, GL_LINK_STATUS, &linked);
            if (!linked)
            {
                std::array<char, 4096> log {};
                gl.GetProgramInfoLog(program, static_cast<GLsizei>(log.size()), nullptr, log.data());
                error = std::string { "OpenGL shader link failed: " } + log.data();
                return false;
            }
            viewport_uniform = gl.GetUniformLocation(program, "viewport_size");
            gl.UseProgram(program);
            gl.Uniform1i(gl.GetUniformLocation(program, "image"), 0);
            gl.GenVertexArrays(1, &vao);
            gl.GenBuffers(1, &vertices);
            gl.GenBuffers(1, &indices);
            gl.GenBuffers(1, &instances);
            gl.BindVertexArray(vao);
            gl.BindBuffer(GL_ARRAY_BUFFER, vertices);
            gl.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
            for (GLuint index = 0; index < 6; ++index)
                gl.EnableVertexAttribArray(index);
            gl.VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(PackedVertex), nullptr);
            // OpenGL encodes buffer byte offsets in pointer arguments; these are never dereferenced.
            // NOLINTBEGIN(performance-no-int-to-ptr)
            gl.VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(PackedVertex), reinterpret_cast<const void*>(sizeof(float) * 2));
            gl.VertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, sizeof(PackedVertex), reinterpret_cast<const void*>(sizeof(float) * 4));
            gl.BindBuffer(GL_ARRAY_BUFFER, instances);
            gl.VertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), nullptr);
            gl.VertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), reinterpret_cast<const void*>(sizeof(float) * 4));
            gl.VertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), reinterpret_cast<const void*>(sizeof(float) * 6));
            // NOLINTEND(performance-no-int-to-ptr)
            for (GLuint index = 3; index < 6; ++index)
                gl.VertexAttribDivisor(index, 1);
            gl.GenTextures(1, &white);
            gl.BindTexture(GL_TEXTURE_2D, white);
            const std::array<std::uint8_t, 4> pixel { 255, 255, 255, 255 };
            gl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixel.data());
            texture_parameters();
            if (const auto* renderer = gl.GetString(GL_RENDERER))
                name += std::string { " / " } + reinterpret_cast<const char*>(renderer);
            int width {}, height {};
            SDL_GetWindowSizeInPixels(window, &width, &height);
            return allocate(screen, { std::max(1, width), std::max(1, height) }, requested_samples);
        }

        void texture_parameters()
        {
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            gl.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }

        bool allocate(Target& target, ViewportSize size, int requested)
        {
            if (size.width < 1 || size.height < 1 || size.width > maximum_size || size.height > maximum_size)
            {
                error = "Render target dimensions exceed the device limits";
                return false;
            }
            for (int samples = std::clamp(requested, 1, maximum_samples); samples >= 1; samples = samples > 2 ? 2 : 1)
            {
                target.size = size;
                target.samples = samples;
                gl.GenTextures(1, &target.texture);
                gl.BindTexture(GL_TEXTURE_2D, target.texture);
                texture_parameters();
                gl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size.width, size.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
                gl.GenFramebuffers(1, &target.resolve);
                gl.BindFramebuffer(GL_FRAMEBUFFER, target.resolve);
                gl.FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, target.texture, 0);
                bool complete = gl.CheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
                target.draw = target.resolve;
                if (complete && samples > 1)
                {
                    gl.GenFramebuffers(1, &target.draw);
                    gl.BindFramebuffer(GL_FRAMEBUFFER, target.draw);
                    gl.GenRenderbuffers(1, &target.color_buffer);
                    gl.BindRenderbuffer(GL_RENDERBUFFER, target.color_buffer);
                    gl.RenderbufferStorageMultisample(GL_RENDERBUFFER, samples, GL_RGBA8, size.width, size.height);
                    GLint actual_samples {};
                    gl.GetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_SAMPLES, &actual_samples);
                    target.samples = std::max(1, actual_samples);
                    gl.FramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, target.color_buffer);
                    complete = gl.CheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
                }
                if (complete && gl.GetError() == GL_NO_ERROR)
                {
                    if (target.samples != requested)
                        core::log_warning("requested {}x MSAA; this render target uses {}x", requested, target.samples);
                    return true;
                }
                release(target);
                while (gl.GetError() != GL_NO_ERROR)
                {
                }
                if (samples == 1)
                    break;
            }
            error = "The OpenGL framebuffer could not be allocated";
            return false;
        }

        Target* active()
        {
            if (!active_target)
                return &screen;
            const auto found = targets.find(active_target);
            return found == targets.end() ? nullptr : &found->second;
        }

        void bind_active()
        {
            SDL_GL_MakeCurrent(window, context);
            auto* target = active();
            if (!target)
                return;
            gl.BindFramebuffer(GL_FRAMEBUFFER, target->draw);
            gl.Viewport(0, 0, target->size.width, target->size.height);
            gl.Disable(GL_DEPTH_TEST);
            gl.Disable(GL_CULL_FACE);
            gl.Disable(GL_SCISSOR_TEST);
            gl.Enable(GL_BLEND);
            gl.Enable(GL_MULTISAMPLE);
            gl.BlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
            gl.UseProgram(program);
            gl.Uniform2f(viewport_uniform, static_cast<float>(target->size.width), static_cast<float>(target->size.height));
            gl.BindVertexArray(vao);
            gl.ActiveTexture(GL_TEXTURE0);
        }

        void clear(const Color& color)
        {
            bind_active();
            gl.ClearColor(color.red * color.alpha, color.green * color.alpha, color.blue * color.alpha, color.alpha);
            gl.Clear(GL_COLOR_BUFFER_BIT);
        }

        void resolve(const Target& target)
        {
            if (target.samples > 1)
            {
                gl.BindFramebuffer(GL_READ_FRAMEBUFFER, target.draw);
                gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, target.resolve);
                gl.Disable(GL_SCISSOR_TEST);
                gl.BlitFramebuffer(0, 0, target.size.width, target.size.height, 0, 0, target.size.width, target.size.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
            }
        }

        GLuint texture_for(const std::shared_ptr<const TexturePixels>& pixels)
        {
            if (!pixels)
                return white;
            if (const auto found = textures.find(pixels.get()); found != textures.end())
            {
                if (!found->second.pixels.expired())
                    return found->second.name;
                gl.DeleteTextures(1, &found->second.name);
                textures.erase(found);
            }
            if (pixels->width < 1 || pixels->height < 1 || pixels->width > maximum_size || pixels->height > maximum_size || pixels->rgba.size() != static_cast<std::size_t>(pixels->width) * static_cast<std::size_t>(pixels->height) * 4)
            {
                error = "Invalid RGBA texture payload";
                return 0;
            }
            GLuint texture {};
            gl.GenTextures(1, &texture);
            gl.BindTexture(GL_TEXTURE_2D, texture);
            gl.PixelStorei(GL_UNPACK_ALIGNMENT, 1);
            gl.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, pixels->width, pixels->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels->rgba.data());
            texture_parameters();
            if (gl.GetError() != GL_NO_ERROR)
            {
                error = "OpenGL texture upload failed";
                gl.DeleteTextures(1, &texture);
                return 0;
            }
            textures.emplace(pixels.get(), Texture { pixels, texture });
            return texture;
        }

        // One draw call: a run of consecutive meshes that share a texture and scissor and are
        // not instanced, or a single instanced mesh.
        struct Batch
        {
            GLuint texture {};
            std::optional<MeshClip> clip;
            std::size_t first_index {}, index_count {}, first_instance {}, instance_count { 1 };
        };
        std::vector<PackedVertex> packed_vertices;
        std::vector<GLuint> packed_indices;
        std::vector<PackedInstance> packed_instances;
        std::vector<Batch> batches;

        static bool same_clip(const std::optional<MeshClip>& a, const std::optional<MeshClip>& b)
        {
            return a.has_value() == b.has_value() && (!a || (a->x == b->x && a->y == b->y && a->width == b->width && a->height == b->height));
        }

        void point_instances(std::size_t first)
        {
            // OpenGL encodes buffer byte offsets in pointer arguments; these are never dereferenced.
            // NOLINTBEGIN(performance-no-int-to-ptr)
            const auto offset = first * sizeof(PackedInstance);
            gl.VertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), reinterpret_cast<const void*>(offset));
            gl.VertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), reinterpret_cast<const void*>(offset + sizeof(float) * 4));
            gl.VertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(PackedInstance), reinterpret_cast<const void*>(offset + sizeof(float) * 6));
            // NOLINTEND(performance-no-int-to-ptr)
        }

        // Uploads a whole compiled list once and draws it with as few calls as painter order
        // allows; per-mesh uploads made the driver synchronise hundreds of times a frame.
        void draw(const std::vector<IndexedMesh>& meshes)
        {
            packed_vertices.clear();
            packed_indices.clear();
            packed_instances.clear();
            batches.clear();
            // Instance zero is the identity transform read by every uninstanced draw.
            packed_instances.push_back({ 0, 0, 1, 1, 1, 0, 1, 1, 1, 1 });
            for (const auto& mesh : meshes)
            {
                if (mesh.vertices.empty() || mesh.indices.empty())
                    continue;
                if (std::any_of(mesh.indices.begin(), mesh.indices.end(), [&](int index)
                        {
                            return index < 0 || static_cast<std::size_t>(index) >= mesh.vertices.size();
                        }))
                {
                    error = "Invalid index in drawing mesh";
                    continue;
                }
                if (packed_vertices.size() + mesh.vertices.size() > static_cast<std::size_t>(std::numeric_limits<GLuint>::max()) || packed_indices.size() + mesh.indices.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()) || packed_instances.size() + mesh.instances.size() > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max()))
                    break;
                const auto textured = mesh.texture != nullptr;
                const auto texture = textured ? texture_for(mesh.texture) : white;
                if (!texture)
                    continue;
                const auto base = static_cast<GLuint>(packed_vertices.size());
                for (const auto& vertex : mesh.vertices)
                {
                    const auto u = textured ? static_cast<float>(vertex.uv.x) : -1.0f, v = textured ? static_cast<float>(vertex.uv.y) : -1.0f;
                    packed_vertices.push_back({ static_cast<float>(vertex.position.x), static_cast<float>(vertex.position.y), u, v, vertex.color.red, vertex.color.green, vertex.color.blue, vertex.color.alpha });
                }
                const auto first_index = packed_indices.size();
                for (const auto index : mesh.indices)
                    packed_indices.push_back(base + static_cast<GLuint>(index));
                if (mesh.instances.empty())
                {
                    // Batch::texture stays zero until a textured mesh joins the run.
                    auto* run = batches.empty() ? nullptr : &batches.back();
                    if (run && run->first_instance == 0 && same_clip(run->clip, mesh.clip) && (!textured || !run->texture || run->texture == texture))
                    {
                        run->index_count += mesh.indices.size();
                        if (textured)
                            run->texture = texture;
                    }
                    else
                        batches.push_back({ textured ? texture : 0, mesh.clip, first_index, mesh.indices.size(), 0, 1 });
                    continue;
                }
                const auto first_instance = packed_instances.size();
                for (const auto& item : mesh.instances)
                    packed_instances.push_back({ static_cast<float>(item.translation.x), static_cast<float>(item.translation.y), static_cast<float>(item.scale.x), static_cast<float>(item.scale.y), std::cos(item.rotation), std::sin(item.rotation), item.tint.red * item.tint.alpha, item.tint.green * item.tint.alpha, item.tint.blue * item.tint.alpha, item.tint.alpha });
                batches.push_back({ textured ? texture : 0, mesh.clip, first_index, mesh.indices.size(), first_instance, mesh.instances.size() });
            }
            if (batches.empty())
                return;
            gl.BindBuffer(GL_ARRAY_BUFFER, vertices);
            gl.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(packed_vertices.size() * sizeof(PackedVertex)), packed_vertices.data(), GL_STREAM_DRAW);
            gl.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices);
            gl.BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(packed_indices.size() * sizeof(GLuint)), packed_indices.data(), GL_STREAM_DRAW);
            gl.BindBuffer(GL_ARRAY_BUFFER, instances);
            gl.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(packed_instances.size() * sizeof(PackedInstance)), packed_instances.data(), GL_STREAM_DRAW);
            GLuint bound_texture = 0;
            std::size_t bound_instance = std::numeric_limits<std::size_t>::max();
            bool scissor_known = false;
            std::optional<MeshClip> bound_clip;
            for (const auto& batch : batches)
            {
                const auto texture = batch.texture ? batch.texture : white;
                if (texture != bound_texture)
                {
                    gl.BindTexture(GL_TEXTURE_2D, texture);
                    bound_texture = texture;
                }
                if (!scissor_known || !same_clip(bound_clip, batch.clip))
                {
                    if (batch.clip)
                    {
                        gl.Enable(GL_SCISSOR_TEST);
                        gl.Scissor(batch.clip->x, active()->size.height - batch.clip->y - batch.clip->height, std::max(0, batch.clip->width), std::max(0, batch.clip->height));
                    }
                    else
                        gl.Disable(GL_SCISSOR_TEST);
                    bound_clip = batch.clip;
                    scissor_known = true;
                }
                if (batch.first_instance != bound_instance)
                {
                    point_instances(batch.first_instance);
                    bound_instance = batch.first_instance;
                }
                // NOLINTNEXTLINE(performance-no-int-to-ptr)
                gl.DrawElementsInstanced(GL_TRIANGLES, static_cast<GLsizei>(batch.index_count), GL_UNSIGNED_INT, reinterpret_cast<const void*>(batch.first_index * sizeof(GLuint)), static_cast<GLsizei>(batch.instance_count));
            }
        }
    };

    GlRenderDevice::GlRenderDevice(std::unique_ptr<Impl> implementation) : impl_(std::move(implementation))
    {
    }
    GlRenderDevice::~GlRenderDevice() = default;

    RenderDevicePtr GlRenderDevice::create(SDL_Window* window, bool vertical_sync, int samples)
    {
        if (!window)
            return nullptr;
        auto implementation = std::make_unique<Impl>();
        implementation->window = window;
        implementation->requested_samples = std::clamp(samples, 1, 16);
        implementation->context = SDL_GL_CreateContext(window);
        if (!implementation->context)
        {
            core::log_warning("OpenGL context is unavailable: {}", SDL_GetError());
            return nullptr;
        }
        if (!SDL_GL_MakeCurrent(window, implementation->context) || !implementation->initialize())
        {
            core::log_warning("OpenGL renderer is unavailable: {}", implementation->error);
            return nullptr;
        }
        auto result = std::unique_ptr<GlRenderDevice> { new GlRenderDevice { std::move(implementation) } };
        result->set_vertical_sync(vertical_sync);
        core::log_info("render device ready on {}", result->backend_name());
        return result;
    }

    std::string_view GlRenderDevice::backend_name() const
    {
        return impl_->name;
    }
    ViewportSize GlRenderDevice::drawable_size() const
    {
        if (impl_->active_target)
            if (auto* target = impl_->active())
                return target->size;
        int width {}, height {};
        SDL_GetWindowSizeInPixels(impl_->window, &width, &height);
        return { std::max(1, width), std::max(1, height) };
    }
    float GlRenderDevice::display_scale() const
    {
        return std::max(1.0f, SDL_GetWindowDisplayScale(impl_->window));
    }
    void GlRenderDevice::set_vertical_sync(bool enabled)
    {
        if (!SDL_GL_SetSwapInterval(enabled ? 1 : 0))
            core::log_warning("OpenGL vertical sync could not be changed: {}", SDL_GetError());
    }
    void GlRenderDevice::begin_frame(const Color& clear)
    {
        SDL_GL_MakeCurrent(impl_->window, impl_->context);
        impl_->active_target = 0;
        const auto size = drawable_size();
        if (!impl_->screen.draw || impl_->screen.size.width != size.width || impl_->screen.size.height != size.height)
        {
            impl_->release(impl_->screen);
            if (!impl_->allocate(impl_->screen, size, impl_->requested_samples))
                return;
        }
        for (auto entry = impl_->textures.begin(); entry != impl_->textures.end();)
            if (entry->second.pixels.expired())
            {
                impl_->gl.DeleteTextures(1, &entry->second.name);
                entry = impl_->textures.erase(entry);
            }
            else
                ++entry;
        impl_->clear(clear);
    }
    void GlRenderDevice::submit(const DrawList& list)
    {
        if (!impl_->active() || !impl_->active()->draw)
            return;
        impl_->bind_active();
        // The anti-aliasing band tracks the monitor's pixel density so edge softness stays
        // constant in logical pixels instead of sharpening on high-DPI displays.
        DrawCompileOptions options;
        options.feather = std::clamp(display_scale(), 1.0f, 4.0f);
        options.multisampled = impl_->active()->samples > 1;
        const auto compiled = compile_draw_list(list, [&](Vec2 position, std::string_view text, Color color, float scale)
            {
                return impl_->font.build(position, text, color, scale);
            },
            options);
        impl_->draw(compiled.meshes);
        impl_->gl.Disable(GL_SCISSOR_TEST);
    }
    void GlRenderDevice::end_frame()
    {
        if (!impl_->screen.draw)
            return;
        impl_->resolve(impl_->screen);
        impl_->gl.BindFramebuffer(GL_READ_FRAMEBUFFER, impl_->screen.resolve);
        impl_->gl.BindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        impl_->gl.Disable(GL_SCISSOR_TEST);
        const auto size = impl_->screen.size;
        impl_->gl.BlitFramebuffer(0, 0, size.width, size.height, 0, 0, size.width, size.height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        SDL_GL_SwapWindow(impl_->window);
        if (const auto error = impl_->gl.GetError(); error != GL_NO_ERROR)
        {
            impl_->error = "OpenGL frame failed with error " + std::to_string(error);
            core::log_error("{}", impl_->error);
        }
        impl_->active_target = 0;
    }
    std::optional<RenderTarget> GlRenderDevice::create_render_target(ViewportSize size, int samples)
    {
        SDL_GL_MakeCurrent(impl_->window, impl_->context);
        Impl::Target target;
        if (!impl_->allocate(target, size, std::clamp(samples, 1, 16)))
        {
            impl_->bind_active();
            return std::nullopt;
        }
        const auto id = impl_->next_target++;
        const auto actual_samples = target.samples;
        impl_->targets.emplace(id, target);
        impl_->bind_active();
        return RenderTarget { id, size, actual_samples };
    }
    bool GlRenderDevice::begin_target(const RenderTarget& target, const Color& clear)
    {
        if (impl_->targets.find(target.id) == impl_->targets.end())
        {
            impl_->error = "Render target is no longer valid";
            return false;
        }
        impl_->active_target = target.id;
        impl_->clear(clear);
        return true;
    }
    std::optional<TexturePixels> GlRenderDevice::read_target(const RenderTarget& target)
    {
        SDL_GL_MakeCurrent(impl_->window, impl_->context);
        const auto found = impl_->targets.find(target.id);
        if (found == impl_->targets.end())
        {
            impl_->error = "Render target is no longer valid";
            return std::nullopt;
        }
        const auto& resource = found->second;
        impl_->resolve(resource);
        impl_->gl.BindFramebuffer(GL_READ_FRAMEBUFFER, resource.resolve);
        TexturePixels pixels;
        pixels.width = resource.size.width;
        pixels.height = resource.size.height;
        const auto row = static_cast<std::size_t>(pixels.width) * 4;
        pixels.rgba.resize(row * static_cast<std::size_t>(pixels.height));
        impl_->gl.PixelStorei(GL_PACK_ALIGNMENT, 1);
        impl_->gl.ReadPixels(0, 0, pixels.width, pixels.height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.rgba.data());
        if (impl_->gl.GetError() != GL_NO_ERROR)
        {
            impl_->error = "OpenGL render target readback failed";
            impl_->bind_active();
            return std::nullopt;
        }
        for (int y = 0; y < pixels.height / 2; ++y)
            std::swap_ranges(pixels.rgba.begin() + static_cast<std::ptrdiff_t>(row * static_cast<std::size_t>(y)), pixels.rgba.begin() + static_cast<std::ptrdiff_t>(row * static_cast<std::size_t>(y + 1)), pixels.rgba.begin() + static_cast<std::ptrdiff_t>(row * static_cast<std::size_t>(pixels.height - y - 1)));
        impl_->bind_active();
        return pixels;
    }
    void GlRenderDevice::destroy_render_target(const RenderTarget& target)
    {
        SDL_GL_MakeCurrent(impl_->window, impl_->context);
        const auto found = impl_->targets.find(target.id);
        if (found == impl_->targets.end())
            return;
        if (impl_->active_target == target.id)
            impl_->active_target = 0;
        impl_->release(found->second);
        impl_->targets.erase(found);
        impl_->bind_active();
    }
    void GlRenderDevice::resume_frame()
    {
        impl_->active_target = 0;
        impl_->bind_active();
    }
    bool GlRenderDevice::load_font(const std::filesystem::path& path)
    {
        return impl_->font.load(path);
    }
    float GlRenderDevice::measure_text_width(std::string_view text, float scale) const
    {
        return impl_->font.measure(text, scale);
    }
    float GlRenderDevice::text_line_height(float scale) const
    {
        return impl_->font.line_height(scale);
    }
    const std::string& GlRenderDevice::last_error() const
    {
        return impl_->error;
    }
    void* GlRenderDevice::native_context() const
    {
        return impl_->context;
    }
    int GlRenderDevice::multisample_count() const
    {
        return impl_->screen.samples;
    }
} // namespace rigidbodies::render
