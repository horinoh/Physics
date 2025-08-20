#include "LinAlg.h"
#include "Mat.h"

const LinAlg::Mat2 LinAlg::Mat2::_Identity = LinAlg::Mat2({ 1.0f, 0.0f }, { 0.0f, 1.0f });
const LinAlg::Mat3 LinAlg::Mat3::_Identity = LinAlg::Mat3({ 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f });
const LinAlg::Mat4 LinAlg::Mat4::_Identity = LinAlg::Mat4({ 1.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 1.0f });

const LinAlg::Mat2 LinAlg::Mat2::_Zero = LinAlg::Mat2({ 0.0f, 0.0f }, { 0.0f, 0.0f });
const LinAlg::Mat3 LinAlg::Mat3::_Zero = LinAlg::Mat3({ 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f });
const LinAlg::Mat4 LinAlg::Mat4::_Zero = LinAlg::Mat4({ 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f });