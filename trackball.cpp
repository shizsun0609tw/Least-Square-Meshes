/* 
 *  Simple trackball-like motion adapted (ripped off) from projtex.c
 *  (written by David Yu and David Blythe).  See the SIGGRAPH '96
 *  Advanced OpenGL course notes.
 *
 *
 *  Usage:
 *  
 *  o  call tbInit() in before any other tb call
 *  o  call tbReshape() from the reshape callback
 *  o  call tbMatrix() to get the trackball matrix rotation
 *  o  call tbStartMotion() to begin trackball movememt
 *  o  call tbStopMotion() to stop trackball movememt
 *  o  call tbMotion() from the motion callback
 *  o  call tbAnimate(GL_TRUE) if you want the trackball to continue 
 *     spinning after the mouse button has been released
 *  o  call tbAnimate(GL_FALSE) if you want the trackball to stop 
 *     spinning after the mouse button has been released
 *
 *  Typical setup:
 *
 *
 */

/* includes */
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <memory.h>
#include <Windows.h>
#include <numeric>
#include <iostream>
#include <map>
#include <set>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseLU>

#include "glut/glut.h"
#include "mtxlib.h"
#include "glm.h"
#include "trackball.h"

using namespace std;

/* globals */
static GLuint    tb_lasttime;
static GLfloat   tb_lastposition[3];

static GLfloat   tb_angle = 0.0;
static GLfloat   tb_axis[3];
static GLfloat   tb_transform[4][4];

static GLuint    tb_width;
static GLuint    tb_height;

static GLint     tb_button = -1;
static GLboolean tb_tracking = GL_FALSE;
static GLboolean tb_animate = GL_TRUE;

_GLMmodel* dinoMesh;
_GLMmodel* recMesh;

vector<int> featureList;
int selectedFeature = -1;
int last_x, last_y;

/* functions */
static void _tbPointToVector(int x, int y, int width, int height, float v[3])
{
  float d, a;

  /* project x, y onto a hemi-sphere centered within width, height. */
  v[0] = (2.0 * x - width) / width;
  v[1] = (height - 2.0 * y) / height;
  d = sqrt(v[0] * v[0] + v[1] * v[1]);
  v[2] = cos((3.14159265 / 2.0) * ((d < 1.0) ? d : 1.0));
  a = 1.0 / sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] *= a;
  v[1] *= a;
  v[2] *= a;
}

static void _tbAnimate(void)
{
  glutPostRedisplay();
}

void _tbStartMotion(int x, int y, int button, int time)
{
  assert(tb_button != -1);

  tb_tracking = GL_TRUE;
  tb_lasttime = time;
  _tbPointToVector(x, y, tb_width, tb_height, tb_lastposition);
}

void _tbStopMotion(int button, unsigned time)
{
  assert(tb_button != -1);

  tb_tracking = GL_FALSE;

  if (time == tb_lasttime && tb_animate) {
    glutIdleFunc(_tbAnimate);
  } else {
    tb_angle = 0.0;
    if (tb_animate)
      glutIdleFunc(0);
  }
}

void tbAnimate(GLboolean animate)
{
  tb_animate = animate;
}

void tbInit(GLuint button)
{
  tb_button = button;
  tb_angle = 0.0;

  /* put the identity in the trackball transform */
  glPushMatrix();
  glLoadIdentity();
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();
}

void tbMatrix()
{
  assert(tb_button != -1);

  glPushMatrix();
  glLoadIdentity();
  glRotatef(tb_angle, tb_axis[0], tb_axis[1], tb_axis[2]);
  glMultMatrixf((GLfloat *)tb_transform);
  glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
  glPopMatrix();

  glMultMatrixf((GLfloat *)tb_transform);
}

void gettbMatrix(float *m)
{
	glPushMatrix();
	glLoadIdentity();
	glRotatef(tb_angle, tb_axis[0], tb_axis[1], tb_axis[2]);
	glMultMatrixf((GLfloat *)tb_transform);
	glGetFloatv(GL_MODELVIEW_MATRIX, (GLfloat *)tb_transform);
	glPopMatrix();

	memcpy(m , tb_transform , 16 * sizeof(float));
}

void tbReshape(int width, int height)
{
  assert(tb_button != -1);

  tb_width  = width;
  tb_height = height;
}

void tbMouse(int button, int state, int x, int y)
{
  assert(tb_button != -1);

  if (state == GLUT_DOWN && button == tb_button)
    _tbStartMotion(x, y, button, glutGet(GLUT_ELAPSED_TIME));
  else if (state == GLUT_UP && button == tb_button)
    _tbStopMotion(button, glutGet(GLUT_ELAPSED_TIME));
}

void tbMotion(int x, int y)
{
  GLfloat current_position[3], dx, dy, dz;

  assert(tb_button != -1);

  if (tb_tracking == GL_FALSE)
    return;

  _tbPointToVector(x, y, tb_width, tb_height, current_position);

  /* calculate the angle to rotate by (directly proportional to the
     length of the mouse movement */
  dx = current_position[0] - tb_lastposition[0];
  dy = current_position[1] - tb_lastposition[1];
  dz = current_position[2] - tb_lastposition[2];
  tb_angle = 90.0 * sqrt(dx * dx + dy * dy + dz * dz);

  /* calculate the axis of rotation (cross product) */
  tb_axis[0] = tb_lastposition[1] * current_position[2] - 
               tb_lastposition[2] * current_position[1];
  tb_axis[1] = tb_lastposition[2] * current_position[0] - 
               tb_lastposition[0] * current_position[2];
  tb_axis[2] = tb_lastposition[0] * current_position[1] - 
               tb_lastposition[1] * current_position[0];

  /* reset for next time */
  tb_lasttime = glutGet(GLUT_ELAPSED_TIME);
  tb_lastposition[0] = current_position[0];
  tb_lastposition[1] = current_position[1];
  tb_lastposition[2] = current_position[2];

  /* remember to draw new position */
  glutPostRedisplay();
}

vector3 Unprojection(vector2 _2Dpos)
{
    float Depth;
    int viewport[4];
    double ModelViewMatrix[16];				//Model_view matrix
    double ProjectionMatrix[16];			//Projection matrix

    glPushMatrix();
    tbMatrix();

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, ModelViewMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX, ProjectionMatrix);

    glPopMatrix();

    glReadPixels((int)_2Dpos.x, viewport[3] - (int)_2Dpos.y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &Depth);

    double X = _2Dpos.x;
    double Y = _2Dpos.y;
    double wpos[3] = { 0.0 , 0.0 , 0.0 };

    gluUnProject(X, ((double)viewport[3] - Y), (double)Depth, ModelViewMatrix, ProjectionMatrix, viewport, &wpos[0], &wpos[1], &wpos[2]);

    return vector3(wpos[0], wpos[1], wpos[2]);
}

/************************************************************************************************/
/*                                           functions                                          */
/************************************************************************************************/
set<int> RandomSelectIndices(_GLMmodel* originMesh)
{
    set<int> selectedIndices;
    int selectedVerticesNum = 1000;

    srand(time(NULL));

    while (selectedIndices.size() < selectedVerticesNum)
    {
        selectedIndices.insert(rand() % originMesh->numvertices + 1);
    }

    return selectedIndices;
}

map<int, set<int>> GetConnectMap(_GLMmodel* originMesh)
{
    map<int, set<int>> res;

    for (int i = 0; i < originMesh->numtriangles; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            res.insert(pair<int, set<int>>(originMesh->triangles[i].vindices[j], set<int>()));

            res[originMesh->triangles[i].vindices[j]].insert(originMesh->triangles[i].vindices[(j + 1) % 3]);
            res[originMesh->triangles[i].vindices[j]].insert(originMesh->triangles[i].vindices[(j + 2) % 3]);
        }
    }

    return res;
}
Eigen::MatrixXd ExeLUSolver(Eigen::SparseMatrix<double> *A, Eigen::SparseMatrix<double> *b)
{
    Eigen::SparseLU<Eigen::SparseMatrix<double>> solver(*A);

    cout << "---------------------------------------------------------" << endl;
    cout << "Start to solve green dino by LU solver" << endl;
    cout << "---------------------------------------------------------" << endl;
  
    clock_t start, end;

    start = clock();
    Eigen::MatrixXd res = solver.solve(*b);
    end = clock();

    cout << "solve cost: " << (float)(end - start) / CLK_TCK << "s" << endl;

    return res;
}

Eigen::MatrixXd ExeCholeskySolver(Eigen::SparseMatrix<double> *A, Eigen::SparseMatrix<double> *b)
{
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(*A);

    cout << "---------------------------------------------------------" << endl;
    cout << "Start to solve red dino by Cholesky solver" << endl;
    cout << "---------------------------------------------------------" << endl;

    clock_t start, end;

    start = clock();
    Eigen::MatrixXd res = solver.solve(*b);
    end = clock();

    cout << "solve cost: " << (float)(end - start) / CLK_TCK << "s" << endl;

    return res;
}

vector<double> LeastSquareSolver(set<int> controlIndices, map<int, set<int>> connectedMap, _GLMmodel* originMesh, int solverFlag)
{
    // Solve Ax = b
    Eigen::SparseMatrix<double> A(originMesh->numvertices + controlIndices.size(), originMesh->numvertices);
    Eigen::SparseMatrix<double> b(originMesh->numvertices + controlIndices.size(), 3);
    vector<Eigen::Triplet<double>> tripletListA, tripletListb;
    Eigen::Triplet<double> value;

    // Laplacian
    int row = 0;
    for (auto iterMap = connectedMap.begin(); iterMap != connectedMap.end(); ++iterMap, ++row)
    {
        int col = row;

        tripletListA.push_back(Eigen::Triplet<double>(row, col, 1));
        for (auto iterSet = (*iterMap).second.begin(); iterSet != (*iterMap).second.end(); ++iterSet)
        {
            col = (*iterSet) - 1;

            value = Eigen::Triplet<double>(row, col, -1.0f / (double)(*iterMap).second.size());
            tripletListA.push_back(value);
        }
    }
    
    // Constraint
    row = 0;
    for (auto iter = controlIndices.begin(); iter != controlIndices.end(); ++iter, ++row)
    {
        value = Eigen::Triplet<double>(originMesh->numvertices + row, *iter - 1, 1);
        tripletListA.push_back(value);
        
        for (int i = 0; i < 3; ++i)
        {
            value = Eigen::Triplet<double>(originMesh->numvertices + row, i, originMesh->vertices[*iter * 3 + i]);
            tripletListb.push_back(value);
        }
    }
    
    A.setFromTriplets(tripletListA.begin(), tripletListA.end());
    b.setFromTriplets(tripletListb.begin(), tripletListb.end());
    
    Eigen::SparseMatrix<double> ATA = A.transpose() * A;
    b = A.transpose() * b;
    
    Eigen::MatrixXd x;

    if (solverFlag == 0)
    {
        x = ExeLUSolver(&ATA, &b);
    }
    else
    {
        x = ExeCholeskySolver(&ATA, &b);
    }

    vector<double> res;

    for (int i = 0; i < x.rows(); ++i)
    {
        for (int j = 0; j < x.cols(); ++j)
        {
            res.push_back(x.row(i).col(j).value());
        }
    }

    return res;
}

_GLMmodel* ReconstructModel(_GLMmodel* recMesh, vector<double> solveVertices)
{
    for (int i = 0; i < solveVertices.size(); ++i)
    {
        recMesh->vertices[i + 3] = solveVertices[i];
    }

    return recMesh;
}

float CalFittingError(_GLMmodel* originMesh, vector<double> solveVertices)
{
    double sum = 0;
    for (int i = 0; i < solveVertices.size(); ++i)
    {
        sum += (originMesh->vertices[i + 3] - solveVertices[i]) * (originMesh->vertices[i + 3] - solveVertices[i]);
    }

    return sqrt(sum);
}

void init(void)
{
    tbInit(GLUT_LEFT_BUTTON);
    tbAnimate(GL_TRUE);

    dinoMesh = glmReadOBJ("../data/dino.obj");
    recMesh = glmReadOBJ("../data/dino.obj");

    glmUnitize(dinoMesh);
    glmFacetNormals(dinoMesh);
    glmVertexNormals(dinoMesh, 90.0f);

    set<int> selectedIndices = RandomSelectIndices(dinoMesh);
    map<int, set<int>> connectedMap = GetConnectMap(dinoMesh);

    // LU Solver
    vector<double> solveVertices = LeastSquareSolver(selectedIndices, connectedMap, dinoMesh, 0);
    cout << "LU solver fitting error: " << CalFittingError(dinoMesh, solveVertices) << endl; 
    recMesh = ReconstructModel(recMesh, solveVertices);
    cout << "\n\n\n" << endl;

    // Cholesky Solver
    solveVertices = LeastSquareSolver(selectedIndices, connectedMap, dinoMesh, 1);
    cout << "Cholesky solver fitting error: " << CalFittingError(dinoMesh, solveVertices) << endl;
    dinoMesh = ReconstructModel(dinoMesh, solveVertices);

    glmUnitize(recMesh);
    glmUnitize(dinoMesh);
    glmFacetNormals(recMesh);
    glmFacetNormals(dinoMesh);
    glmVertexNormals(recMesh, 90.0f);
    glmVertexNormals(dinoMesh, 90.0f);
}

void reshape(int width, int height)
{
    int base = min(width, height);

    tbReshape(width, height);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0f, (GLdouble)width / (GLdouble)height, 1.0f, 128.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0f, 0.0f, -3.5f);
}

void RenderMesh(GLMmodel* model, vector3 translate, vector3 color)
{
    glPushMatrix();

    tbMatrix();

    glTranslatef(translate.x, translate.y, translate.z);
    glRotatef(-75, 1, 0, 0);
    glRotatef(-45, 0, 0, 1);

    glEnable(GL_LIGHTING);
    glColor3f(1.0f, 1.0f, 1.0f);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glmDraw(model, GLM_SMOOTH);

    glPolygonOffset(1.0f, 1.0f);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glLineWidth(1.0f);
    glColor3f(color.x, color.y, color.z);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glmDraw(model, GLM_SMOOTH);

    glPointSize(10.0f);
    glColor3f(1.0f, 0.0f, 0.0f);
    glDisable(GL_LIGHTING);

    glBegin(GL_POINTS);
    for (int i = 0; i < featureList.size(); ++i)
    {
        int idx = featureList[i];
        glVertex3fv((float*)&model->vertices[3 * idx]);
    }
    glEnd();

    glPopMatrix();
}

void display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    RenderMesh(dinoMesh, vector3(0.8, 0.0, 0.0), vector3(0.6, 0.0, 0.0));
    RenderMesh(recMesh, vector3(-0.2, 0.0, 0.0), vector3(0.0, 0.6, 0.0));

    glFlush();
    glutSwapBuffers();
}

void mouse(int button, int state, int x, int y)
{
    tbMouse(button, state, x, y);

    return;
}

void motion(int x, int y)
{
    tbMotion(x, y);

    if (selectedFeature != -1)
    {
        matrix44 m;
        vector4 vec = vector4((float)(x - last_x) / 100.0f, (float)(y - last_y) / 100.0f, 0.0f, 1.0f);

        gettbMatrix((float*)&m);
        vec = m * vec;

        vec.y *= -1;
    }

    last_x = x;
    last_y = y;
}

void timf(int value)
{
    glutPostRedisplay();
    glutTimerFunc(1, timf, 0);
}

int main(int argc, char** argv)
{
    int WindWidth = 400;
    int WindHeight = 400;

    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_position[] = { 0.0, 0.0, 1.0, 0.0 };

    glutInit(&argc, argv);
    glutInitWindowSize(WindWidth, WindHeight);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutCreateWindow("Trackball");

    init();

    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glClearColor(0, 0, 0, 0);

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);

    glEnable(GL_LIGHT0);
    glDepthFunc(GL_LESS);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_COLOR_MATERIAL);
    
    glutTimerFunc(40, timf, 0);


    glutMainLoop();

    return 0;
}
