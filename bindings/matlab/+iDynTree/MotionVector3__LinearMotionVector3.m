classdef MotionVector3__LinearMotionVector3 < iDynTree.GeomVector3__LinearMotionVector3
  methods
    function self = MotionVector3__LinearMotionVector3(varargin)
      self@iDynTree.GeomVector3__LinearMotionVector3('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(275, varargin{:});
        tmp = iDynTreeMATLAB_wrap(275, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = cross(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(276, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(277, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
