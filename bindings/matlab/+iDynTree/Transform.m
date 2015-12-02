classdef Transform < SwigRef
  methods
    function self = Transform(varargin)
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(519, varargin{:});
        tmp = iDynTreeMATLAB_wrap(519, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = getSemantics(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(520, self, varargin{:});
    end
    function varargout = getRotation(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(521, self, varargin{:});
    end
    function varargout = getPosition(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(522, self, varargin{:});
    end
    function varargout = setRotation(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(523, self, varargin{:});
    end
    function varargout = setPosition(self,varargin)
      [varargout{1:nargout}] = iDynTreeMATLAB_wrap(524, self, varargin{:});
    end
    function varargout = inverse(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(527, self, varargin{:});
    end
    function varargout = mtimes(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(528, self, varargin{:});
    end
    function varargout = asHomogeneousTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(530, self, varargin{:});
    end
    function varargout = asAdjointTransform(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(531, self, varargin{:});
    end
    function varargout = asAdjointTransformWrench(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(532, self, varargin{:});
    end
    function varargout = log(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(533, self, varargin{:});
    end
    function varargout = toString(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(534, self, varargin{:});
    end
    function varargout = display(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(535, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(536, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(525, varargin{:});
    end
    function varargout = inverse2(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(526, varargin{:});
    end
    function varargout = Identity(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(529, varargin{:});
    end
  end
end
