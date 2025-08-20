#include "LinAlg.h"
#include "Vec.h"

const LinAlg::Vec2 LinAlg::Vec2::_Zero = LinAlg::Vec2(0.0f);
const LinAlg::Vec3 LinAlg::Vec3::_Zero = LinAlg::Vec3(0.0f);
const LinAlg::Vec4 LinAlg::Vec4::_Zero = LinAlg::Vec4(0.0f);

const LinAlg::Vec2 LinAlg::Vec2::_One = LinAlg::Vec2(1.0f);
const LinAlg::Vec3 LinAlg::Vec3::_One = LinAlg::Vec3(1.0f);
const LinAlg::Vec4 LinAlg::Vec4::_One = LinAlg::Vec4(1.0f);

const LinAlg::Vec2 LinAlg::Vec2::_AxisX = LinAlg::Vec2(1.0f, 0.0f);
const LinAlg::Vec3 LinAlg::Vec3::_AxisX = LinAlg::Vec3(1.0f, 0.0f, 0.0f);
const LinAlg::Vec4 LinAlg::Vec4::_AxisX = LinAlg::Vec4(1.0f, 0.0f, 0.0f, 0.0f);

const LinAlg::Vec2 LinAlg::Vec2::_AxisY = LinAlg::Vec2(0.0f, 1.0f);
const LinAlg::Vec3 LinAlg::Vec3::_AxisY = LinAlg::Vec3(0.0f, 1.0f, 0.0f);
const LinAlg::Vec4 LinAlg::Vec4::_AxisY = LinAlg::Vec4(0.0f, 1.0f, 0.0f, 0.0f);

const LinAlg::Vec3 LinAlg::Vec3::_AxisZ = LinAlg::Vec3(0.0f, 0.0f, 1.0f);
const LinAlg::Vec4 LinAlg::Vec4::_AxisZ = LinAlg::Vec4(0.0f, 0.0f, 1.0f, 0.0f);

const LinAlg::Vec4 LinAlg::Vec4::_AxisW = LinAlg::Vec4(0.0f, 0.0f, 0.0f, 1.0f);

const LinAlg::Vec2 LinAlg::Vec2::_Epsilon = LinAlg::Vec2((std::numeric_limits<float>::epsilon)());
const LinAlg::Vec3 LinAlg::Vec3::_Epsilon = LinAlg::Vec3((std::numeric_limits<float>::epsilon)());
const LinAlg::Vec4 LinAlg::Vec4::_Epsilon = LinAlg::Vec4((std::numeric_limits<float>::epsilon)());

const LinAlg::Vec2 LinAlg::Vec2::_Min = LinAlg::Vec2((std::numeric_limits<float>::min)());
const LinAlg::Vec3 LinAlg::Vec3::_Min = LinAlg::Vec3((std::numeric_limits<float>::min)());
const LinAlg::Vec4 LinAlg::Vec4::_Min = LinAlg::Vec4((std::numeric_limits<float>::min)());

const LinAlg::Vec2 LinAlg::Vec2::_Max = LinAlg::Vec2((std::numeric_limits<float>::max)());
const LinAlg::Vec3 LinAlg::Vec3::_Max = LinAlg::Vec3((std::numeric_limits<float>::max)());
const LinAlg::Vec4 LinAlg::Vec4::_Max = LinAlg::Vec4((std::numeric_limits<float>::max)());

const LinAlg::Vec2 LinAlg::Vec2::_UnitXY = LinAlg::Vec2(1.0f / std::sqrtf(2.0f));
const LinAlg::Vec3 LinAlg::Vec3::_UnitXYZ = LinAlg::Vec3(1.0f / std::sqrtf(3.0f));
const LinAlg::Vec4 LinAlg::Vec4::_UnitXYZW = LinAlg::Vec4(1.0f / std::sqrtf(4.0f));
