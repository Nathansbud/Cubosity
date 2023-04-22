#include "UIUtil.h"

namespace UIUtil {
    QDoubleSpinBox* makeDoubleSpinBox(
        double min,
        double max,
        double step,
        double val,
        int decimal
    ) {
        QDoubleSpinBox* box = new QDoubleSpinBox();
        box->setMinimum(min);
        box->setMaximum(max);
        box->setSingleStep(step);
        box->setValue(val);
        box->setDecimals(decimal);
        return box;
    }
}
