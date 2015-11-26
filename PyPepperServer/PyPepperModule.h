/*
 *  PyPepperModule.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 18/11/15.
 *  Copyright 2010, 2015 Galaxy Network. All rights reserved.
 *
 */

#ifndef PY_PEPPER_MODULE_H
#define PY_PEPPER_MODULE_H

#include <PyModuleStub.h>

namespace pyride {

class PyPepperModule : public PyModuleExtension
{
public:
  static PyPepperModule * instance();
  
private:
  static PyPepperModule * s_pyPepperModule;

  PyPepperModule();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_PEPPER_MODULE_H
