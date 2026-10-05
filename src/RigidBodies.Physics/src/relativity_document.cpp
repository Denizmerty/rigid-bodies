#include <rigidbodies/physics/relativity_document.hpp>

#include <cmath>
#include <stdexcept>

namespace rigidbodies::physics
{
    namespace
    {
        using Json = content::Json;

        constexpr std::string_view relativity_key = "relativity";
        constexpr std::string_view rest_mass_key = "rest_mass_kg";
        constexpr std::string_view speed_fraction_key = "speed_fraction_c";

        // Shown when a file is rejected on open, so each names the field and its accepted range.
        constexpr const char* missing_object_error = "A document that requires special_relativity must contain a relativity object.";
        constexpr const char* rest_mass_error = "relativity.rest_mass_kg must be a finite number from 0.000001 to 1000000.";
        constexpr const char* speed_fraction_error = "relativity.speed_fraction_c must be 0 or a finite number from 0.000000000001 to 0.9999999.";

        Real bounded_number(const Json& object, std::string_view key, Real minimum, Real maximum, const char* error)
        {
            const auto* value = object.find(key);
            if (value == nullptr || !value->is_number())
                throw std::invalid_argument(error);
            const auto number = value->as_number();
            if (!std::isfinite(number) || number < minimum || number > maximum)
                throw std::invalid_argument(error);
            return number;
        }
    }

    bool declares_special_relativity(const content::Json& root)
    {
        const auto* features = root.find("required_features");
        if (features == nullptr || !features->is_array())
            return false;
        for (const auto& feature : features->as_array())
            if (feature.is_string() && feature.as_string() == special_relativity_feature)
                return true;
        return false;
    }

    std::optional<RelativitySetup> read_relativity_setup(const content::Json& root)
    {
        if (!declares_special_relativity(root))
            return std::nullopt;
        const auto* object = root.find(relativity_key);
        if (object == nullptr || !object->is_object())
            throw std::invalid_argument(missing_object_error);
        RelativitySetup setup;
        setup.rest_mass_kg = bounded_number(*object, rest_mass_key, minimum_relativity_mass_kg, maximum_relativity_mass_kg, rest_mass_error);
        setup.speed_fraction = bounded_number(*object, speed_fraction_key, 0.0, maximum_speed_fraction, speed_fraction_error);
        // A moving speed is never slower than the slowest one a learner can set.
        if (!settable_speed_fraction(setup.speed_fraction))
            throw std::invalid_argument(speed_fraction_error);
        // A written −0 is rest, and must fingerprint like the rest the session writes.
        if (setup.speed_fraction == 0.0)
            setup.speed_fraction = 0.0;
        return setup;
    }

    content::Json encode_relativity_setup(const RelativitySetup& setup)
    {
        if (!valid_relativity_setup(setup))
            throw std::invalid_argument("A relativity setup needs a rest mass within the relativity bounds and a speed from 0 c to the maximum");
        // A negative zero is rest, so neither the file nor the fingerprint ever holds "-0".
        const auto speed_fraction = setup.speed_fraction == 0.0 ? 0.0 : setup.speed_fraction;
        return Json::Object { { std::string(rest_mass_key), setup.rest_mass_kg }, { std::string(speed_fraction_key), speed_fraction } };
    }

    void write_relativity_setup(content::Json& root, const RelativitySetup& setup)
    {
        auto encoded = encode_relativity_setup(setup);
        if (const auto* existing = root.find(relativity_key); existing != nullptr && existing->is_object())
        {
            auto merged = *existing;
            for (auto& member : encoded.as_object())
                merged[member.first] = std::move(member.second);
            encoded = std::move(merged);
        }
        root[std::string(relativity_key)] = std::move(encoded);
    }

    std::string relativity_setup_fingerprint(const RelativitySetup& setup)
    {
        return content::write_json(encode_relativity_setup(setup));
    }
}
