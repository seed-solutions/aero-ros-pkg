#ifndef AERO_CONTROLLER_AJOINT_INDEX_H_
#define AERO_CONTROLLER_AJOINT_INDEX_H_

namespace aero
{
  namespace controller
  {

    class AJointIndex
    {
    public: AJointIndex(size_t _id, size_t _sidx, size_t _ridx, std::string _name) :
	id(_id), stroke_index(_sidx), raw_index(_ridx), joint_name(_name)
      {
      }

    public: AJointIndex(const AJointIndex& _aji)
      {
	id = _aji.id;
	stroke_index = _aji.stroke_index;
	raw_index = _aji.raw_index;
	joint_name = _aji.joint_name;
      }

    public: AJointIndex& operator=(const AJointIndex& _aji)
      {
	id = _aji.id;
	stroke_index = _aji.stroke_index;
	raw_index = _aji.raw_index;
	joint_name = _aji.joint_name;
	return *this;
      }

    public: size_t id;

    public: size_t stroke_index;

    public: size_t raw_index;

    public: std::string joint_name;
    };

  }
}

#endif
