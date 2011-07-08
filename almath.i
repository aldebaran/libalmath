%module almath

%feature("autodoc", "1");

%{
#include "almath/types/alaxismask.h"

#include "almath/types/alpose2d.h"
#include "almath/types/alposition2d.h"
#include "almath/types/alposition3d.h"
#include "almath/types/alposition6d.h"
#include "almath/types/alpositionandvelocity.h"

#include "almath/types/alrotation.h"
#include "almath/types/alrotation3d.h"

#include "almath/types/altransform.h"

#include "almath/types/alvelocity3d.h"
#include "almath/types/alvelocity6d.h"

#include "almath/types/altransformandvelocity6d.h"

#include "almath/tools/aldubinscurve.h"
#include "almath/tools/altrigonometry.h"

#include "almath/tools/altransformhelpers.h"
#include "almath/tools/almath.h"
%}

%include "almath/types/alaxismask.h"

%include "almath/types/alpose2d.h"
%include "almath/types/alposition2d.h"
%include "almath/types/alposition3d.h"
%include "almath/types/alposition6d.h"
%include "almath/types/alpositionandvelocity.h"

%include "almath/types/alrotation.h"
%include "almath/types/alrotation3d.h"

%include "almath/types/altransform.h"

%include "almath/types/alvelocity3d.h"
%include "almath/types/alvelocity6d.h"

%include "almath/types/altransformandvelocity6d.h"

%include "almath/tools/aldubinscurve.h"
%include "almath/tools/altrigonometry.h"

%include "almath/tools/altransformhelpers.h"
%include "almath/tools/almath.h"


%extend AL::Math::Pose2D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "x = %g, y = %g, theta = %g",
                      $self->x, $self->y, $self->theta);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Pose2D___str__($self);
   }
};

%extend AL::Math::Position2D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "x = %g, y = %g",
                      $self->x, $self->y);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Position2D___str__($self);
   }
};


%extend AL::Math::Position3D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "x = %g, y = %g, z = %g",
                      $self->x, $self->y, $self->z);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Position3D___str__($self);
   }
};


%extend AL::Math::Position6D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "x = %g, y = %g, z = %g, wx = %g, wy = %g, wz = %g",
                    $self->x, $self->y, $self->z,
                    $self->wx, $self->wy, $self->wz);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Position6D___str__($self);
   }
};

%extend AL::Math::PositionAndVelocity {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "q = %g, dq = %g",
                    $self->q, $self->dq);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_PositionAndVelocity___str__($self);
   }
};


%extend AL::Math::Rotation {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "[[%g, %g, %g]\n"
                    " [%g, %g, %g]\n"
                    " [%g, %g, %g]]",
                      $self->r1_c1, $self->r1_c2, $self->r1_c3,
                      $self->r2_c1, $self->r2_c2, $self->r2_c3,
                      $self->r3_c1, $self->r3_c2, $self->r3_c3);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Rotation___str__($self);
   }
};


%extend AL::Math::Rotation3D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "wx = %g, wy = %g, wz = %g",
                      $self->wx, $self->wy, $self->wz);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Rotation3D___str__($self);
   }
};


%extend AL::Math::Transform {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "[[%g, %g, %g, %g]\n"
                    " [%g, %g, %g, %g]\n"
                    " [%g, %g, %g, %g]]",
                      $self->r1_c1, $self->r1_c2, $self->r1_c3, $self->r1_c4,
                      $self->r2_c1, $self->r2_c2, $self->r2_c3, $self->r2_c4,
                      $self->r3_c1, $self->r3_c2, $self->r3_c3, $self->r3_c4);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Transform___str__($self);
   }
};


%extend AL::Math::TransformAndVelocity6D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "[[%g, %g, %g, %g]\n"
                    " [%g, %g, %g, %g]\n"
                    " [%g, %g, %g, %g]]\n"
                    "[xd = %g, yd = %g, zd = %g, wxd = %g, wyd = %g, wzd = %g]",
                      $self->T.r1_c1, $self->T.r1_c2, $self->T.r1_c3, $self->T.r1_c4,
                      $self->T.r2_c1, $self->T.r2_c2, $self->T.r2_c3, $self->T.r2_c4,
                      $self->T.r3_c1, $self->T.r3_c2, $self->T.r3_c3, $self->T.r3_c4,
                      $self->V.xd,  $self->V.yd,  $self->V.zd,
                      $self->V.wxd, $self->V.wyd, $self->V.wzd);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_TransformAndVelocity6D___str__($self);
   }
};

%extend AL::Math::Velocity3D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "xd = %g, yd = %g, zd = %g",
                      $self->xd, $self->yd, $self->zd);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Velocity3D___str__($self);
   }
};

%extend AL::Math::Velocity6D {
   char *__str__() {
       static char tmp[1024];
       sprintf(tmp, "xd = %g, yd = %g, zd = %g, wxd = %g, wyd = %g, wzd = %g",
                    $self->xd,  $self->yd,  $self->zd,
                    $self->wxd, $self->wyd, $self->wzd);
       return tmp;
   }

   char *__repr__() {
        return AL_Math_Velocity6D___str__($self);
   }
};


%include "std_vector.i"
namespace std {
   %template(vectorFloat) vector<float>;
   %template(vectorPosition2D) vector<AL::Math::Position2D>;
   %template(vectorPose2D) vector<AL::Math::Pose2D>;
   %template(vectorPosition6D) vector<AL::Math::Position6D>;
}

