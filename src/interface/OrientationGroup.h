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

    QPushButton* addRow;
    QPushButton* removeRow;

    QDoubleSpinBox* lambda;

    MatrixInput* rotInput;

    Eigen::MatrixXd rotation;

    QColor color;
    int groupID;

    inline static int nextID = 0;
    inline static std::map<int, std::pair<std::string, MatrixXd>> presets;

    static void populatePresets() {
        MatrixXd octahedron(4, 3);
        octahedron << 0.57735, -0.57735, 0.57735,
                      -0.57735, -0.57735, 0.57735,
                      -0.57735, 0.57735, 0.57735,
                      0.57735, 0.57735, 0.57735;


        presets.insert({0, {"Octahedron", octahedron}});

        MatrixXd dodecahedron(6, 3);
        dodecahedron << 0.850651, 0, 0.525731,
                        0.850651, 1.23058e-07, -0.525731,
                        -0.850651, 0, -0.525731,
                        -0.850651, 1.23058e-07, 0.525731,
                        -0.525731, 0.850651, -1.23058e-07,
                        0.525731, 0.850651, 1.23058e-07;

        presets.insert({1, {"Dodecahedron", dodecahedron}});
    }

    void setPreset(int preset) {
        MatrixXd& newOrientation = presets[preset].second;
        if(this->rotation.rows() < newOrientation.rows()) {
            for(int i = 0; i < newOrientation.rows() - this->rotation.rows(); i++) {
                this->rotInput->insertInputRow(this->rotation.rows() + i);
            }
        } else {
            for(int i = 0; i < this->rotation.rows() - newOrientation.rows(); i++) {
                this->rotInput->removeRow(this->rotation.rows() - i);
            }
        }

        this->rotation = newOrientation;
        this->rotInput->populateValues(this->rotation);
    }

    OrientationGroup(QColor color, QWidget *parent = nullptr) {
        this->color = color;
        this->colorButton = new QPushButton("");
        this->colorButton->setStyleSheet(
            QString("background-color: %1; border: none;").arg(color.name())
        );

        this->addButton = new QPushButton("+");

        this->addRow = new QPushButton("R+");
        this->removeRow = new QPushButton("R-");

        this->lambda = UIUtil::makeDoubleSpinBox(0, 10, 0.01, 1.0, 2);
        this->rotation = Matrix3d::Identity();
        this->rotInput = new MatrixInput(rotation, 3, 3);

        this->contents = new QHBoxLayout();
        this->contents->addWidget(addButton);
        this->contents->addWidget(colorButton);
        this->contents->addWidget(lambda);
        this->contents->addWidget(rotInput);
        this->contents->addWidget(addRow);
        this->contents->addWidget(removeRow);
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

        connect(this->addRow, &QPushButton::clicked, this, [&] {
            this->rotation.conservativeResize(this->rotation.rows() + 1, Eigen::NoChange);
            this->rotation.row(this->rotation.rows() - 1) << 0, 0, 1;
            this->rotInput->insertInputRow(this->rotation.rows() - 1);
        });

        connect(this->removeRow, &QPushButton::clicked, this, [&] {
            if(this->rotation.rows() > 3) {
                this->rotation.conservativeResize(this->rotation.rows() - 1, Eigen::NoChange);
                this->rotInput->removeRow(this->rotation.rows());
            }
        });

        this->groupID = nextID++;
    }
signals:
    void colorChanged();
    void makeActive(int group);
};
