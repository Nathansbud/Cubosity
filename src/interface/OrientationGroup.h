#pragma once
#include "UIUtil.h"
#include "MatrixInput.h"

#include <QGroupBox>
#include <QBoxLayout>
#include <QColor>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QString>
#include <QColorDialog>
#include <Eigen/Dense>

#include <iostream>

class OrientationGroup : public QGroupBox
{
    Q_OBJECT
public:
    QHBoxLayout* contents;
    QPushButton* addButton;
    QPushButton* colorButton;
    QDoubleSpinBox* lambda;

    MatrixInput* rotInput;

    Eigen::MatrixXd rotation;

    QColor color;
    int groupID;

    inline static int nextID = 0;

    OrientationGroup(QColor color, QWidget *parent = nullptr) {
        this->color = color;
        this->colorButton = new QPushButton("");
        this->colorButton->setStyleSheet(
            QString("background-color: %1; border: none;").arg(color.name())
        );

        this->addButton = new QPushButton("+");

        this->lambda = UIUtil::makeDoubleSpinBox(0.01, 10, 0.01, 1.0, 2);
        this->rotation = Matrix3d::Identity();
        this->rotInput = new MatrixInput(rotation, 3, 3);

        this->contents = new QHBoxLayout();
        this->contents->addWidget(addButton);
        this->contents->addWidget(colorButton);
        this->contents->addWidget(lambda);
        this->contents->addWidget(rotInput);
        this->setLayout(contents);

        connect(this->addButton, &QPushButton::clicked, this, [&]() { emit makeActive(this->groupID); });
        connect(this->colorButton, &QPushButton::clicked, this, [&]() {
            QColor ret = QColorDialog::getColor();
            if (ret.isValid()) {
                this->colorButton->setStyleSheet(
                    QString("background-color: %1; border: none;").arg(ret.name())
                );

                this->color = ret;
                emit colorChanged();
            }
        });

        connect(this->rotInput, &MatrixInput::cellModified, this, [&](int r, int c) {
            this->rotation(r, c) = static_cast<QLineEdit*>(this->rotInput->cellWidget(r, c))->text().toDouble();
        });

        this->groupID = nextID++;
    }
signals:
    void colorChanged();
    void makeActive(int group);
};
