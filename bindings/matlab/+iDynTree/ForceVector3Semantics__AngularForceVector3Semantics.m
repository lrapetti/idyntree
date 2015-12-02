classdef ForceVector3Semantics__AngularForceVector3Semantics < iDynTree.GeomVector3Semantics__AngularForceVector3Semantics
  methods
    function self = ForceVector3Semantics__AngularForceVector3Semantics(varargin)
      self@iDynTree.GeomVector3Semantics__AngularForceVector3Semantics('_swigCreate');
      if nargin~=1 || ~ischar(varargin{1}) || ~strcmp(varargin{1},'_swigCreate')
        % How to get working on C side? Commented out, replaed by hack below
        %self.swigInd = iDynTreeMATLAB_wrap(271, varargin{:});
        tmp = iDynTreeMATLAB_wrap(271, varargin{:}); % FIXME
        self.swigInd = tmp.swigInd;
        tmp.swigInd = uint64(0);
      end
    end
    function delete(self)
      if self.swigInd
        iDynTreeMATLAB_wrap(274, self);
        self.swigInd=uint64(0);
      end
    end
  end
  methods(Static)
    function varargout = compose(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(272, varargin{:});
    end
    function varargout = inverse(varargin)
      [varargout{1:max(1,nargout)}] = iDynTreeMATLAB_wrap(273, varargin{:});
    end
  end
end
