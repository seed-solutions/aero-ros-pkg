#ifndef AERO_CONTROLLER_AJOINT_INDEX_H_
#define AERO_CONTROLLER_AJOINT_INDEX_H_

namespace aero
{
  namespace controller
  {

  /// @brief Assignment of CAN bus ID, stroke index, raw index, and joint name
    class AJointIndex
    {
      /// @brief constructor
      /// @param _id CAN bus ID
      /// @param _sidx Index in stroke vector
      /// @param _ridx Index in raw data buffer
      /// @param _name Joint name
    public: AJointIndex(size_t _id, size_t _sidx, size_t _ridx, std::string _name) :
	id(_id), stroke_index(_sidx), raw_index(_ridx), joint_name(_name)
      {
      }

      /// @brief copy constructor
    public: AJointIndex(const AJointIndex& _aji)
      {
	id = _aji.id;
	stroke_index = _aji.stroke_index;
	raw_index = _aji.raw_index;
	joint_name = _aji.joint_name;
      }

      /// @brief = operator
    public: AJointIndex& operator=(const AJointIndex& _aji)
      {
	id = _aji.id;
	stroke_index = _aji.stroke_index;
	raw_index = _aji.raw_index;
	joint_name = _aji.joint_name;
	return *this;
      }

      /// @brief CAN bus ID
    public: size_t id;

      /// @brief Index in stroke vector
    public: size_t stroke_index;

      /// @brief Index in raw data buffer
    public: size_t raw_index;

      /// @brief Joint name
    public: std::string joint_name;
    };

  }
}

#endif
