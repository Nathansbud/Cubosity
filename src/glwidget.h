#pragma once

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#include "arap.h"
#include "graphics/camera.h"
#include "graphics/shader.h"

#include <QOpenGLWidget>
#include <QElapsedTimer>
#include <QTimer>
#include <memory>

class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

    // extra controls, dispatching to mesh/ARAP
    void simplify();
    void expand();
    void expandAll();

    void subdivide();
    void denoise();
    void cubify();

    void saveMesh(const std::string& filePath) {
        m_arap.mesh.saveMesh(filePath);
    };

    void reset();

    // Orientation group callback~
    void updateVertexColors();

    Settings settings;
private:
    static const int FRAMES_TO_AVERAGE = 30;

    Eigen::Vector3f transformToWorldRay(int x, int y);

    // Basic OpenGL Overrides
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override;

    // Event Listeners
    void mousePressEvent  (QMouseEvent *event) override;
    void mouseMoveEvent   (QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent       (QWheelEvent *event) override;
    void keyPressEvent    (QKeyEvent   *event) override;
    void keyReleaseEvent  (QKeyEvent   *event) override;

private slots:
    // Physics Tick
    void tick();

private:
    ARAP    m_arap;
    Camera  m_camera;
    Shader *m_defaultShader;
    Shader *m_pointShader;
    Shader *m_textureShader;

    float m_movementScaling;
    float m_vertexSelectionThreshold;
    float m_vSize;

    // Timing
    QElapsedTimer m_deltaTimeProvider; // For measuring elapsed time
    QTimer        m_intervalTimer;     // For triggering timed events

    // Movement
    int m_forward;
    int m_sideways;
    int m_vertical;

    // Mouse handler stuff
    int m_lastX;
    int m_lastY;
    bool m_leftCapture;
    bool m_rightCapture;
    SelectMode m_rightClickSelectMode;
    int m_lastSelectedVertex = -1;

    bool simplifying = false;
    bool cubing = false;
    int cubeIter = false;
    bool expanding = false;
};
