#pragma once
#include <QDoubleSpinBox>

namespace UIUtil {
    extern QDoubleSpinBox* makeDoubleSpinBox(
        double min,
        double max,
        double step,
        double val,
        int decimal
    );
}
