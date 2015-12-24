using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using Grpc.Core.Utils;

namespace Bodyskeleton
{

    public class BodySkeletonImpl : Bodyskeleton.BodySkeleton.IBodySkeleton
    {
        readonly Dictionary<JointType, Tuple<Bodyskeleton.Joint, JointType>> joints
            = new Dictionary<JointType, Tuple<Joint, JointType>>();
        bool user_found;
        User user_count = new User
        {
            Id = 0
        };

        public BodySkeletonImpl()
        {
            joints.Add(JointType.SpineBase,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "torso",
                        Parent = "root",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.SpineBase));
            joints.Add(JointType.SpineMid,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "chest",
                        Parent = "torso",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.SpineMid));
            joints.Add(JointType.Neck,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "neck",
                        Parent = "chest2",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.Neck));
            joints.Add(JointType.Head,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "head",
                        Parent = "neck",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.Head));
            joints.Add(JointType.ShoulderLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_shoulder",
                        Parent = "chest2",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.ElbowLeft));
            joints.Add(JointType.ElbowLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_elbow",
                        Parent = "left_shoulder",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.WristLeft));
            joints.Add(JointType.WristLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_hand",
                        Parent = "left_elbow",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandLeft));
            joints.Add(JointType.HandLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_hand1",
                        Parent = "left_hand",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandLeft));
            joints.Add(JointType.ShoulderRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_shoulder",
                        Parent = "chest2",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.ElbowRight));
            joints.Add(JointType.ElbowRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_elbow",
                        Parent = "right_shoulder",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.WristRight));
            joints.Add(JointType.WristRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_hand",
                        Parent = "right_elbow",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandRight));
            joints.Add(JointType.HandRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_hand1",
                        Parent = "right_hand",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandRight));
            joints.Add(JointType.HipLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_hip",
                        Parent = "torso",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.KneeLeft));
            joints.Add(JointType.KneeLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_knee",
                        Parent = "left_hip",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.AnkleLeft));
            joints.Add(JointType.AnkleLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_foot",
                        Parent = "left_knee",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.AnkleLeft));
            joints.Add(JointType.FootLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_foot1",
                        Parent = "left_foot",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.FootLeft));
            joints.Add(JointType.HipRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_hip",
                        Parent = "torso",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.KneeRight));
            joints.Add(JointType.KneeRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_knee",
                        Parent = "right_hip",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.AnkleRight));
            joints.Add(JointType.AnkleRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_foot",
                        Parent = "right_knee",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.AnkleRight));
            joints.Add(JointType.FootRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_foot1",
                        Parent = "right_foot",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.FootRight));
            joints.Add(JointType.SpineShoulder,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "chest2",
                        Parent = "chest",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.SpineShoulder));
            joints.Add(JointType.HandTipLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_hand2",
                        Parent = "left_hand1",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandTipLeft));
            joints.Add(JointType.ThumbLeft,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "left_hand3",
                        Parent = "left_hand1",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.ThumbLeft));
            joints.Add(JointType.HandTipRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_hand2",
                        Parent = "right_hand1",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.HandTipRight));
            joints.Add(JointType.ThumbRight,
                new Tuple<Bodyskeleton.Joint, JointType>(
                    new Bodyskeleton.Joint
                    {
                        Name = "right_hand3",
                        Parent = "right_hand1",
                        Position = new Bodyskeleton.Position { X = 0, Y = 0, Z = 0 },
                        Orientation = new Bodyskeleton.Orientation { X = 0, Y = 0, Z = 0, W = 0 }
                    }, JointType.ThumbRight));
        }

        public async Task GetSkeleton(Bodyskeleton.User request,
            Grpc.Core.IServerStreamWriter<Bodyskeleton.Joint> responseStream,
            Grpc.Core.ServerCallContext context)
        {
            if (!this.user_found) return;

            foreach (Tuple<Bodyskeleton.Joint, JointType> response in joints.Values)
            {
                await responseStream.WriteAsync(response.Item1);
            }

            this.user_found = false;
        }

        public Task<User> GetUsers(User request, Grpc.Core.ServerCallContext context)
        {
            return Task.FromResult(GetUserCount(request));
        }

        private User GetUserCount(User user)
        {
            return this.user_count;
        }

        public void SetUserCount(int _user_count)
        {
            this.user_count.Id = _user_count;
        }

        public void SetJoint(int _user_id,
            IReadOnlyDictionary<JointType, Microsoft.Kinect.Joint> _joints,
            IReadOnlyDictionary<JointType, JointOrientation> _orientations)
        {
            this.user_found = true;

            foreach (JointType jointType in joints.Keys)
            {
                TrackingState trackingState = _joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    joints[jointType].Item1.Position.X = _joints[jointType].Position.X;
                    joints[jointType].Item1.Position.Y = _joints[jointType].Position.Y;
                    joints[jointType].Item1.Position.Z = _joints[jointType].Position.Z;
                    if (_orientations[joints[jointType].Item2].Orientation.W != 0)
                    {
                        joints[jointType].Item1.Orientation.X =
                            _orientations[joints[jointType].Item2].Orientation.X;
                        joints[jointType].Item1.Orientation.Y
                            = _orientations[joints[jointType].Item2].Orientation.Y;
                        joints[jointType].Item1.Orientation.Z =
                            _orientations[joints[jointType].Item2].Orientation.Z;
                        joints[jointType].Item1.Orientation.W =
                            _orientations[joints[jointType].Item2].Orientation.W;
                    }
                    else
                    {
                        joints[jointType].Item1.Orientation.X = 1;
                        joints[jointType].Item1.Orientation.Y = 0;
                        joints[jointType].Item1.Orientation.Z = 0;
                        joints[jointType].Item1.Orientation.W = 1;
                    }
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    joints[jointType].Item1.Position.X = _joints[jointType].Position.X;
                    joints[jointType].Item1.Position.Y = _joints[jointType].Position.Y;
                    joints[jointType].Item1.Position.Z = _joints[jointType].Position.Z;
                    if (_orientations[joints[jointType].Item2].Orientation.W != 0)
                    {
                        joints[jointType].Item1.Orientation.X =
                            _orientations[joints[jointType].Item2].Orientation.X;
                        joints[jointType].Item1.Orientation.Y
                            = _orientations[joints[jointType].Item2].Orientation.Y;
                        joints[jointType].Item1.Orientation.Z =
                            _orientations[joints[jointType].Item2].Orientation.Z;
                        joints[jointType].Item1.Orientation.W =
                            _orientations[joints[jointType].Item2].Orientation.W;
                    }
                    else
                    {
                        joints[jointType].Item1.Orientation.X = 1;
                        joints[jointType].Item1.Orientation.Y = 0;
                        joints[jointType].Item1.Orientation.Z = 0;
                        joints[jointType].Item1.Orientation.W = 1;
                    }
                }
            }
        }

    }
}
