#pragma once

#include <rigidbodies/physics/content_json.hpp>
#include <rigidbodies/physics/special_relativity.hpp>

#include <optional>
#include <string>
#include <string_view>

namespace rigidbodies::physics
{
    // A scenario document that declares this feature carries a top-level "relativity" object:
    // { "rest_mass_kg": <number>, "speed_fraction_c": <number> }, both required. Clock readings are
    // time counters and are never stored.
    inline constexpr std::string_view special_relativity_feature = "special_relativity";
    // True when required_features is an array containing "special_relativity".
    [[nodiscard]] bool declares_special_relativity(const content::Json& root);
    // The validated top-level "relativity" object of a document that declares the feature, or
    // std::nullopt when the feature is not declared (a "relativity" object without the feature is an
    // ordinary retained extension and is ignored). Throws std::invalid_argument naming the field when
    // the feature is declared and the object is missing or invalid.
    [[nodiscard]] std::optional<RelativitySetup> read_relativity_setup(const content::Json& root);
    // Throws std::invalid_argument unless valid_relativity_setup(setup), so nothing writes a setup
    // that the reader would reject. A negative zero speed is written as 0.
    [[nodiscard]] content::Json encode_relativity_setup(const RelativitySetup& setup);
    // Replaces root["relativity"], keeping unknown members of an existing object (additive minor revisions).
    void write_relativity_setup(content::Json& root, const RelativitySetup& setup);
    // Exact text for the session's setup fingerprint: the encoded object written with max_digits10.
    [[nodiscard]] std::string relativity_setup_fingerprint(const RelativitySetup& setup);
}
