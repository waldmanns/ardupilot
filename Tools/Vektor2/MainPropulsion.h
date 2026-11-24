#pragma once

#include <stdint.h>
#include <new>

namespace Vektor2 {

// Permanent routing IDs: never renumber these when adding a backend.
enum class ComponentId : uint8_t {
    VSP1 = 0, VSP2 = 1, ThrusterBow = 2, ThrusterStern = 3,
    AZI1 = 4, AZI2 = 5, Count
};

enum class MainPropulsionType : uint8_t { VSP = 0, Azipod = 1, None = 255 };
enum class AzipodLayout : uint8_t { Unspecified = 0, AFT = 1 };

constexpr bool is_vsp(ComponentId id)
{
    return id == ComponentId::VSP1 || id == ComponentId::VSP2;
}
constexpr bool is_azipod(ComponentId id)
{
    return id == ComponentId::AZI1 || id == ComponentId::AZI2;
}
constexpr bool is_tunnel_thruster(ComponentId id)
{
    return id == ComponentId::ThrusterBow || id == ComponentId::ThrusterStern;
}
// Layout defines installed drives; extend only with matching runtime/routing support.
constexpr uint8_t azipod_count(AzipodLayout layout)
{
    return layout == AzipodLayout::AFT ? 2 : 0;
}
constexpr bool component_selected(MainPropulsionType type, AzipodLayout layout, ComponentId id)
{
    return is_tunnel_thruster(id) || (type == MainPropulsionType::VSP && is_vsp(id)) ||
        (type == MainPropulsionType::Azipod && is_azipod(id) &&
         uint8_t(id) - uint8_t(ComponentId::AZI1) < azipod_count(layout));
}

// Four runtime slots, despite six stable component IDs. Inactive IDs must
// never alias active data. The sentinel is deliberately outside the array.
constexpr uint8_t runtime_slot_count = 4;
constexpr uint8_t component_runtime_slot(MainPropulsionType type, AzipodLayout layout, ComponentId id)
{
    return !component_selected(type, layout, id) ? runtime_slot_count :
        (is_azipod(id) ? uint8_t(id) - uint8_t(ComponentId::AZI1) : uint8_t(id));
}

// Exactly one runtime backend is constructed at boot. No heap allocation,
// no runtime switching. Storage is max(sizeof(VSP), sizeof(AZI)), not sum.
// A corrupt selector deliberately constructs neither main propulsion family.
template <typename Vsp, typename Azi>
class MainPropulsionStorage {
public:
    MainPropulsionStorage() = default;
    MainPropulsionStorage(const MainPropulsionStorage&) = delete;
    MainPropulsionStorage& operator=(const MainPropulsionStorage&) = delete;
    ~MainPropulsionStorage()
    {
        if (_type == MainPropulsionType::VSP) { _storage.vsp.~Vsp(); }
        if (_type == MainPropulsionType::Azipod) { _storage.azi.~Azi(); }
    }

    void initialize(int16_t type)
    {
        if (_initialized) { return; }
        _initialized = true;
        if (type == 0) {
            new (&_storage.vsp) Vsp{};
            _type = MainPropulsionType::VSP;
        } else if (type == 1) {
            new (&_storage.azi) Azi{};
            _type = MainPropulsionType::Azipod;
        }
    }

    bool initialized() const { return _initialized; }
    MainPropulsionType type() const { return _type; }
    Vsp* vsp() { return _type == MainPropulsionType::VSP ? &_storage.vsp : nullptr; }
    const Vsp* vsp() const { return _type == MainPropulsionType::VSP ? &_storage.vsp : nullptr; }
    Azi* azi() { return _type == MainPropulsionType::Azipod ? &_storage.azi : nullptr; }
    const Azi* azi() const { return _type == MainPropulsionType::Azipod ? &_storage.azi : nullptr; }

private:
    union Storage {
        Vsp vsp;
        Azi azi;
        Storage() {}
        ~Storage() {}
    } _storage;
    MainPropulsionType _type = MainPropulsionType::None;
    bool _initialized = false;
};

} // namespace Vektor2
