classdef Wrench < iDynTree.SpatialForceVector
  methods
    function self = Wrench(varargin)
      self@iDynTree.SpatialForceVector('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(386, varargin{:});
        tmp = iDynTreeMATLAB_wrap(386, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function varargout = plus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(387, self, varargin{:});
    end
    function varargout = minus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(388, self, varargin{:});
    end
    function varargout = uminus(self,varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(389, self, varargin{:});
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(390, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
  end
end
