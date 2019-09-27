classdef BerdyOptions < SwigRef
  methods
    function this = swig_this(self)
      this = iDynTreeMEX(3, self);
    end
    function self = BerdyOptions(varargin)
      if nargin==1 && strcmp(class(varargin{1}),'SwigRef')
        if ~isnull(varargin{1})
          self.swigPtr = varargin{1}.swigPtr;
        end
      else
        tmp = iDynTreeMEX(1566, varargin{:});
        self.swigPtr = tmp.swigPtr;
        tmp.SwigClear();
      end
    end
    function varargout = berdyVariant(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1567, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1568, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsDynamicVariables(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1569, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1570, self, varargin{1});
      end
    end
    function varargout = includeAllJointAccelerationsAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1571, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1572, self, varargin{1});
      end
    end
    function varargout = includeAllJointTorquesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1573, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1574, self, varargin{1});
      end
    end
    function varargout = includeAllNetExternalWrenchesAsSensors(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1575, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1576, self, varargin{1});
      end
    end
    function varargout = includeCoMAccelerometerAsSensor(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1577, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1578, self, varargin{1});
      end
    end
    function varargout = comConstraintLinkIndexVector(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1579, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1580, self, varargin{1});
      end
    end
    function varargout = includeFixedBaseExternalWrench(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1581, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1582, self, varargin{1});
      end
    end
    function varargout = jointOnWhichTheInternalWrenchIsMeasured(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1583, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1584, self, varargin{1});
      end
    end
    function varargout = baseLink(self, varargin)
      narginchk(1, 2)
      if nargin==1
        nargoutchk(0, 1)
        varargout{1} = iDynTreeMEX(1585, self);
      else
        nargoutchk(0, 0)
        iDynTreeMEX(1586, self, varargin{1});
      end
    end
    function varargout = checkConsistency(self,varargin)
      [varargout{1:nargout}] = iDynTreeMEX(1587, self, varargin{:});
    end
    function delete(self)
      if self.swigPtr
        iDynTreeMEX(1588, self);
        self.SwigClear();
      end
    end
  end
  methods(Static)
  end
end
