#include <rigidbodies/physics/scenario.hpp>
#include <rigidbodies/physics/scenario_document.hpp>

#include <algorithm>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        struct Catalogue
        {
            bool attempted { false };
            std::filesystem::path directory;
            std::vector<ScenarioDescription> descriptions;
            std::vector<ScenarioDocument> documents;
            std::string error;
        };

        Catalogue& catalogue()
        {
            static Catalogue value;
            return value;
        }

        void ensure_catalogue()
        {
            if (!catalogue().attempted)
            {
                std::string error;
#ifdef RIGIDBODIES_DEFAULT_SCENARIO_DIRECTORY
                initialize_scenario_catalogue(RIGIDBODIES_DEFAULT_SCENARIO_DIRECTORY, error);
#else
                initialize_scenario_catalogue(std::filesystem::path { "assets" } / "scenarios", error);
#endif
            }
        }

        bool valid_identifier(std::string_view value)
        {
            return !value.empty() && value.size() <= 128 &&
                std::all_of(value.begin(), value.end(), [](char character)
                    {
                        return (character >= 'a' && character <= 'z') || (character >= '0' && character <= '9') || character == '_' || character == '-';
                    });
        }

        void validate_metadata(const ScenarioMetadata& metadata)
        {
            if (!valid_identifier(metadata.id))
                throw std::runtime_error("scenario metadata.id must use 1-128 lowercase letters, digits, underscores or hyphens");
            if (metadata.title.empty() || metadata.summary.empty() || metadata.concepts.empty() || metadata.suggested_order < 0)
                throw std::runtime_error("scenario metadata requires a title, summary, concepts and a nonnegative suggested_order");
            std::set<std::string> concepts;
            for (const auto& concept : metadata.concepts)
                if (concept.empty() || !concepts.insert(concept).second)
                    throw std::runtime_error("scenario concepts must be nonempty and unique");
            std::set<std::string> prerequisites;
            for (const auto& prerequisite : metadata.prerequisites)
                if (!valid_identifier(prerequisite) || !prerequisites.insert(prerequisite).second)
                    throw std::runtime_error("scenario prerequisites must be unique scenario identifiers");
        }

        void validate_prerequisites(const std::vector<ScenarioDocument>& documents)
        {
            std::map<std::string, std::size_t> indices;
            for (std::size_t index = 0; index < documents.size(); ++index)
                if (!indices.emplace(documents[index].metadata.id, index).second)
                    throw std::runtime_error("duplicate scenario identifier: " + documents[index].metadata.id);

            std::vector<int> visits(documents.size(), 0);
            std::function<void(std::size_t)> visit = [&](std::size_t index)
            {
                if (visits[index] == 1)
                    throw std::runtime_error("cyclic scenario prerequisites at: " + documents[index].metadata.id);
                if (visits[index] == 2)
                    return;
                visits[index] = 1;
                for (const auto& prerequisite : documents[index].metadata.prerequisites)
                {
                    const auto found = indices.find(prerequisite);
                    if (found == indices.end())
                        throw std::runtime_error("unknown prerequisite '" + prerequisite + "' for scenario '" + documents[index].metadata.id + "'");
                    visit(found->second);
                }
                visits[index] = 2;
            };
            for (std::size_t index = 0; index < documents.size(); ++index)
                visit(index);
        }
    }

    bool initialize_scenario_catalogue(const std::filesystem::path& directory, std::string& error)
    {
        auto& live = catalogue();
        live.attempted = true;
        try
        {
            if (!std::filesystem::is_directory(directory))
                throw std::runtime_error("scenario directory does not exist: " + directory.u8string());
            std::vector<std::filesystem::path> paths;
            for (const auto& entry : std::filesystem::directory_iterator(directory))
            {
                auto extension = entry.path().extension().u8string();
                std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char character)
                    {
                        return static_cast<char>(character >= 'A' && character <= 'Z' ? character + ('a' - 'A') : character);
                    });
                if (entry.is_regular_file() && extension == ".json")
                    paths.push_back(entry.path());
                if (paths.size() > 256)
                    throw std::runtime_error("scenario catalogue exceeds 256 documents");
            }
            if (paths.empty())
                throw std::runtime_error("scenario directory contains no JSON documents: " + directory.u8string());
            std::sort(paths.begin(), paths.end());

            std::vector<ScenarioDocument> documents;
            std::uintmax_t total_size = 0;
            for (const auto& path : paths)
            {
                const auto size = std::filesystem::file_size(path);
                total_size += size;
                if (size > 8 * 1024 * 1024 || total_size > 64 * 1024 * 1024)
                    throw std::runtime_error("scenario documents exceed the 8 MiB file or 64 MiB catalogue limit");
                std::ifstream input(path, std::ios::binary);
                if (!input)
                    throw std::runtime_error("cannot open scenario: " + path.u8string());
                std::string text(static_cast<std::size_t>(size), '\0');
                input.read(text.data(), static_cast<std::streamsize>(text.size()));
                if (!input || input.peek() != std::ifstream::traits_type::eof())
                    throw std::runtime_error("cannot read complete scenario: " + path.u8string());
                ScenarioDocument document;
                std::string document_error;
                if (!parse_scenario_document(text, document, document_error))
                    throw std::runtime_error(path.filename().u8string() + ": " + document_error);
                validate_metadata(document.metadata);
                World validation;
                if (!populate_world(document, validation, document_error))
                    throw std::runtime_error(path.filename().u8string() + ": " + document_error);
                documents.push_back(std::move(document));
            }
            validate_prerequisites(documents);
            std::sort(documents.begin(), documents.end(), [](const auto& first, const auto& second)
                {
                    if (first.metadata.suggested_order != second.metadata.suggested_order)
                        return first.metadata.suggested_order < second.metadata.suggested_order;
                    return first.metadata.id < second.metadata.id;
                });
            std::vector<ScenarioDescription> descriptions;
            descriptions.reserve(documents.size());
            for (const auto& document : documents)
            {
                const auto& metadata = document.metadata;
                descriptions.push_back({ metadata.id, metadata.title, metadata.summary, metadata.concepts, metadata.prerequisites, metadata.tags, metadata.collection, metadata.level, metadata.hook, metadata.collection_order, metadata.suggested_order, metadata.lab });
            }
            live.directory = std::filesystem::absolute(directory);
            live.descriptions = std::move(descriptions);
            live.documents = std::move(documents);
            live.error.clear();
            error.clear();
            return true;
        }
        catch (const std::exception& failure)
        {
            error = failure.what();
            live.error = error;
            return false;
        }
    }

    const std::filesystem::path& scenario_catalogue_directory()
    {
        ensure_catalogue();
        return catalogue().directory;
    }

    const std::string& scenario_catalogue_error()
    {
        ensure_catalogue();
        return catalogue().error;
    }

    math::Span<const ScenarioDescription> available_scenarios()
    {
        ensure_catalogue();
        return catalogue().descriptions;
    }

    const ScenarioDescription* find_scenario(std::string_view id)
    {
        for (const auto& description : available_scenarios())
            if (description.id == id)
                return &description;
        return nullptr;
    }

    const ScenarioDocument* scenario_document_for_id(std::string_view id)
    {
        ensure_catalogue();
        for (const auto& document : catalogue().documents)
            if (document.metadata.id == id)
                return &document;
        return nullptr;
    }

    bool load_scenario(World& world, std::string_view id)
    {
        if (const auto* document = scenario_document_for_id(id))
            return populate_world(*document, world, catalogue().error);
        auto& live = catalogue();
        live.error = "unknown scenario identifier: " + std::string { id };
        return false;
    }

    std::string_view default_scenario_id()
    {
        const auto descriptions = available_scenarios();
        return descriptions.empty() ? std::string_view {} : descriptions.front().id;
    }
}
